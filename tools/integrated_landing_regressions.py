#!/usr/bin/env python3
"""M0: repeat the common landing tests using genuine local KSP references.

Local-only. No installation, KSP launch, existing checkout modification, network
fetch, test expectation edits or deletion. Detached worktrees are created under
a new output directory and deliberately retained for failure investigation.
"""
import argparse
import hashlib
import json
import pathlib
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET

PINS=(
    "907349e78b3212cf67b30ef4eaea37dd1412e829",
    "718247a9c8e0ddbd5a555267774b9b0900519bc2",
    "03120ee2f6248ce3a7399e4ad1f4afd99cbe3f53",
    "5b0803cd1e962d6f0c67b88d1f4d5aaff526f40e",
)
FILTER=("FullyQualifiedName~LandingPredictionTests"
        "&FullyQualifiedName!~DunaM1DiagnosticsTests"
        "&FullyQualifiedName!~DunaBrakingHandoffTests"
        "&FullyQualifiedName!~AtmosphericLandingDynamicsTests")
TEST_PROJECT="MechJebLibTest/MechJebLibTest.csproj"

def sha256(path):
    h=hashlib.sha256()
    with path.open("rb") as f:
        for b in iter(lambda:f.read(1024*1024),b""):h.update(b)
    return h.hexdigest()

def execute(args,cwd=None,log=None):
    if log:
        with log.open("w",encoding="utf-8") as f:
            f.write(json.dumps(args)+"\n")
            f.flush()
            p=subprocess.run(args,cwd=cwd,stdout=f,stderr=subprocess.STDOUT)
        return p.returncode
    return subprocess.check_output(args,cwd=cwd,text=True,stderr=subprocess.STDOUT).strip()

def read_trx(path):
    root=ET.parse(path).getroot()
    if not root.tag.endswith("TestRun"):raise ValueError("Not a TRX TestRun")
    rows=[]
    for r in root.findall(".//{*}UnitTestResult"):
        name,outcome=r.get("testName"),r.get("outcome")
        if not name or not outcome:raise ValueError("Missing test identity/outcome")
        rows.append({"name":name,"outcome":outcome,
            "duration":r.get("duration"),
            "message":r.findtext(".//{*}ErrorInfo/{*}Message"),
            "stackTrace":r.findtext(".//{*}ErrorInfo/{*}StackTrace")})
    if not rows:raise ValueError("TRX contains no unit tests")
    return sorted(rows,key=lambda r:(r["name"],r["outcome"]))

def signatures(rows):
    return sorted((r["name"],r["outcome"]) for r in rows)

def compare(a,b):
    from collections import Counter
    x,y=Counter(signatures(a)),Counter(signatures(b))
    return {"removedOrChanged":[{"name":n,"outcome":o,"count":c} for (n,o),c in sorted((x-y).items())],
            "addedOrChanged":[{"name":n,"outcome":o,"count":c} for (n,o),c in sorted((y-x).items())]}

def run(repo,ksp,out):
    repo,ksp,out=map(lambda p:pathlib.Path(p).resolve(),(repo,ksp,out))
    if out.exists():raise ValueError("Output directory must not already exist")
    data=ksp/"KSP_x64_Data"
    managed=data/"Managed"
    # Exact 1.12.5 layout from previously executed local test command.
    refs=[managed/"Assembly-CSharp.dll",managed/"UnityEngine.CoreModule.dll"]
    if not all(p.is_file() for p in refs):raise ValueError("Installed KSP managed reference files not found")
    if not (ksp/"KSP_x64.exe").is_file():raise ValueError("Expected actual Windows KSP installation")
    for p in PINS:
        actual=execute(["git","rev-parse","--verify",p+"^{commit}"],repo)
        if actual!=p:raise ValueError("Missing exact pinned commit "+p)
    root=pathlib.Path(execute(["git","rev-parse","--show-toplevel"],repo)).resolve()
    if root!=repo:raise ValueError("Specify repository root")
    # Worktrees and outputs outside source and installed game.
    if out==repo or repo in out.parents or out==ksp or ksp in out.parents:
        raise ValueError("Use output outside repository and KSP")
    info=execute(["dotnet","--info"],repo)
    files=refs+[ksp/"KSP_x64.exe"]
    for name in ("UnityEngine.dll","UnityEngine.PhysicsModule.dll"):
        p=managed/name
        if p.is_file():files.append(p)
    identity=[{"file":p.name,"bytes":p.stat().st_size,"sha256":sha256(p)} for p in files]
    out.mkdir(parents=True)
    result={"schemaVersion":1,"pins":PINS,"filter":FILTER,
            "actualLocalReferences":identity,"dotnetInfo":info,"runs":[],
            "m0Gate":"not_evaluated"}
    def persist():
        (out/"summary.json").write_text(json.dumps(result,indent=2),encoding="utf-8")
    persist()
    for p in PINS:
        print("Testing "+p,flush=True)
        dst=out/p[:8];dst.mkdir()
        wt=dst/"src"
        # Only new isolated worktrees, never checkout/reset current workspace.
        code=execute(["git","worktree","add","--detach",str(wt),p],repo,dst/"worktree.log")
        item={"commit":p,"worktreeExit":code,"worktree":str(wt)}
        result["runs"].append(item);persist()
        if code:
            item["status"]="worktree_failed";persist();continue
        if execute(["git","rev-parse","HEAD"],wt)!=p:
            item["status"]="wrong_commit";persist();continue
        res=dst/"results";res.mkdir()
        args=["dotnet","test",TEST_PROJECT,"-c","Release",
              "-p:KspDir="+str(ksp),"-p:KspData="+str(data),
              "--filter",FILTER,"--logger","trx;LogFileName=landing.trx",
              "--logger","console;verbosity=normal","--results-directory",str(res)]
        item["testExit"]=execute(args,wt,dst/"test.log")
        try:
            rows=read_trx(res/"landing.trx")
            item["status"]="test_results"
            item["testCount"]=len(rows)
            item["passed"]=sum(x["outcome"]=="Passed" for x in rows)
            item["failures"]=[x for x in rows if x["outcome"]!="Passed"]
            (dst/"all-results.json").write_text(json.dumps(rows,indent=2),encoding="utf-8")
            if p==PINS[0]:baseline=rows
            else:
                if result["runs"][0].get("status")=="test_results":
                    item["comparisonToAirlessBaseline"]=compare(baseline,rows)
        except (OSError,ET.ParseError,ValueError) as e:
            item["status"]="missing_or_invalid_results";item["error"]=str(e)
        persist()
    # Verify installed inputs did not change during the comparison.
    result["referenceInputsUnchanged"]=all(sha256(p)==x["sha256"] for p,x in zip(files,identity))
    all_runs=all(x.get("status")=="test_results" for x in result["runs"])
    result["m0Gate"]="reference_inputs_changed" if not result["referenceInputsUnchanged"] else (
        "incomplete" if not all_runs else "await_review_of_actual_results")
    persist()
    return 0 if all_runs and result["referenceInputsUnchanged"] else 1

class Tests(unittest.TestCase):
    def test_extract_failure_and_compare(self):
        text="""<TestRun xmlns="http://microsoft.com/schemas/VisualStudio/TeamTest/2010"><Results>
        <UnitTestResult testName="a" outcome="Failed" duration="0:00:01">
        <Output><ErrorInfo><Message>assertion</Message><StackTrace>line 42</StackTrace></ErrorInfo></Output>
        </UnitTestResult><UnitTestResult testName="b" outcome="Passed"/></Results></TestRun>"""
        with tempfile.TemporaryDirectory() as d:
            p=pathlib.Path(d)/"a.trx";p.write_text(text,encoding="utf-8")
            r=read_trx(p)
            self.assertEqual(r[0]["stackTrace"],"line 42")
            self.assertEqual(compare(r,r),{"removedOrChanged":[],"addedOrChanged":[]})
            t=[dict(r[0],outcome="Passed"),r[1]]
            self.assertEqual(compare(r,t)["removedOrChanged"],[{"name":"a","outcome":"Failed","count":1}])
            self.assertEqual(compare(r,t)["addedOrChanged"],[{"name":"a","outcome":"Passed","count":1}])
    def test_duplicate_outcomes_preserved(self):
        r=[{"name":"theory","outcome":"Passed"}]*2
        self.assertEqual(compare(r,r[:1])["removedOrChanged"][0]["count"],1)
    def test_invalid_results_fail(self):
        with tempfile.TemporaryDirectory() as d:
            p=pathlib.Path(d)/"a.trx";p.write_text("<TestRun/>",encoding="utf-8")
            with self.assertRaises(ValueError):read_trx(p)

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--repo");p.add_argument("--ksp");p.add_argument("--out")
    p.add_argument("--self-test",action="store_true")
    a=p.parse_args()
    if a.self_test:
        r=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(Tests))
        return 0 if r.wasSuccessful() else 1
    if not all((a.repo,a.ksp,a.out)):p.error("Supply --repo, --ksp and new --out")
    return run(a.repo,a.ksp,a.out)
if __name__=="__main__":
    raise SystemExit(main())
