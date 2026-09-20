using System;
using KSP.Localization;
using UnityEngine;

namespace MuMech;

public class CachedLocalizer : MonoBehaviour
{
	[NonSerialized]
	public string MechJebAscentButton1;

	[NonSerialized]
	public string MechJebAscentButton2;

	[NonSerialized]
	public string MechJebAscentButton3;

	[NonSerialized]
	public string MechJebAscentButton4;

	[NonSerialized]
	public string MechJebAscentButton5;

	[NonSerialized]
	public string MechJebAscentButton6;

	[NonSerialized]
	public string MechJebAscentButton7;

	[NonSerialized]
	public string MechJebAscentButton8;

	[NonSerialized]
	public string MechJebAscentButton9;

	[NonSerialized]
	public string MechJebAscentButton10;

	[NonSerialized]
	public string MechJebAscentButton11;

	[NonSerialized]
	public string MechJebAscentButton12;

	[NonSerialized]
	public string MechJebAscentButton13;

	[NonSerialized]
	public string MechJebAscentButton15;

	[NonSerialized]
	public string MechJebAscentButton17;

	[NonSerialized]
	public string MechJebAscentLabel1;

	[NonSerialized]
	public string MechJebAscentLabel2;

	[NonSerialized]
	public string MechJebAscentLabel3;

	[NonSerialized]
	public string MechJebAscentLabel4;

	[NonSerialized]
	public string MechJebAscentLabel5;

	[NonSerialized]
	public string MechJebAscentLabel6;

	[NonSerialized]
	public string MechJebAscentLabel7;

	[NonSerialized]
	public string MechJebAscentLabel8;

	[NonSerialized]
	public string MechJebAscentLabel9;

	[NonSerialized]
	public string MechJebAscentLabel10;

	[NonSerialized]
	public string MechJebAscentLabel11;

	[NonSerialized]
	public string MechJebAscentLabel12;

	[NonSerialized]
	public string MechJebAscentLabel13;

	[NonSerialized]
	public string MechJebAscentLabel14;

	[NonSerialized]
	public string MechJebAscentLabel15;

	[NonSerialized]
	public string MechJebAscentLabel16;

	[NonSerialized]
	public string MechJebAscentLabel17;

	[NonSerialized]
	public string MechJebAscentLabel18;

	[NonSerialized]
	public string MechJebAscentLabel19;

	[NonSerialized]
	public string MechJebAscentLabel20;

	[NonSerialized]
	public string MechJebAscentLabel21;

	[NonSerialized]
	public string MechJebAscentLabel22;

	[NonSerialized]
	public string MechJebAscentLabel23;

	[NonSerialized]
	public string MechJebAscentLabel24;

	[NonSerialized]
	public string MechJebAscentLabel25;

	[NonSerialized]
	public string MechJebAscentLabel26;

	[NonSerialized]
	public string MechJebAscentLabel27;

	[NonSerialized]
	public string MechJebAscentLabel28;

	[NonSerialized]
	public string MechJebAscentLabel29;

	[NonSerialized]
	public string MechJebAscentLabel30;

	[NonSerialized]
	public string MechJebAscentLabel31;

	[NonSerialized]
	public string MechJebAscentLabel32;

	[NonSerialized]
	public string MechJebAscentLabel33;

	[NonSerialized]
	public string MechJebAscentLabel34;

	[NonSerialized]
	public string MechJebAscentLabel35;

	[NonSerialized]
	public string MechJebAscentLabel36;

	[NonSerialized]
	public string MechJebAscentLabel37;

	[NonSerialized]
	public string MechJebAscentLabel38;

	[NonSerialized]
	public string MechJebAscentLabel39;

	[NonSerialized]
	public string MechJebAscentLabel40;

	[NonSerialized]
	public string MechJebAscentLabel41;

	[NonSerialized]
	public string MechJebAscentLabel42;

	[NonSerialized]
	public string MechJebAscentLabel44;

	[NonSerialized]
	public string MechJebAscentAttachAlt;

	[NonSerialized]
	public string MechJebAscentWarnAttachAltHigh;

	[NonSerialized]
	public string MechJebAscentWarnAttachAltLow;

	[NonSerialized]
	public string MechJebAscentLaunchToTargetLan;

	[NonSerialized]
	public string MechJebAscentWarnInvalidTarget;

	[NonSerialized]
	public string MechJebAscentLaunchToLan;

	[NonSerialized]
	public string MechJebAscentLaunchingToTargetLAN;

	[NonSerialized]
	public string MechJebAscentLaunchingToManualLAN;

	[NonSerialized]
	public string MechJebAscentMsg2;

	[NonSerialized]
	public string MechJebAscentHotStaging;

	[NonSerialized]
	public string MechJebAscentDropSolids;

	[NonSerialized]
	public string MechJebAscentLeadTime;

	[NonSerialized]
	public string MechJebAscentCheckbox2;

	[NonSerialized]
	public string MechJebAscentCheckbox3;

	[NonSerialized]
	public string MechJebAscentCheckbox4;

	[NonSerialized]
	public string MechJebAscentCheckbox5;

	[NonSerialized]
	public string MechJebAscentCheckbox6;

	[NonSerialized]
	public string MechJebAscentCheckbox7;

	[NonSerialized]
	public string MechJebAscentCheckbox8;

	[NonSerialized]
	public string MechJebAscentCheckbox9;

	[NonSerialized]
	public string MechJebAscentCheckbox10;

	[NonSerialized]
	public string MechJebAscentCheckbox11;

	[NonSerialized]
	public string MechJebAscentCheckbox12;

	[NonSerialized]
	public string MechJebAscentCheckbox13;

	[NonSerialized]
	public string MechJebAscentCheckbox14;

	[NonSerialized]
	public string MechJebAscentCheckbox15;

	[NonSerialized]
	public string MechJebAscentCheckbox16;

	[NonSerialized]
	public string MechJebAscentCheckbox17;

	[NonSerialized]
	public string MechJebAscentCheckbox18;

	[NonSerialized]
	public string MechJebAscentCheckbox19;

	[NonSerialized]
	public string MechJebAscentCheckbox20;

	[NonSerialized]
	public string MechJebAscentStatus9;

	[NonSerialized]
	public string MechJebAscentStatus10;

	[NonSerialized]
	public string MechJebAscentStatus11;

	[NonSerialized]
	public string MechJebNavBallGuidanceBtn1;

	[NonSerialized]
	public string MechJebNavBallGuidanceBtn2;

	[NonSerialized]
	public string MechJebInfoItemsUnlimitedText;

	[NonSerialized]
	public string MechJebInfoItemsLabel1;

	[NonSerialized]
	public string MechJebInfoItemsShowEmpty;

	[NonSerialized]
	public string MechJebInfoItemsHideEmpty;

	[NonSerialized]
	public string MechJebInfoItemsButton5;

	[NonSerialized]
	public string MechJebInfoItemsButton6;

	[NonSerialized]
	public string MechJebInfoItemsStatsColumn0;

	[NonSerialized]
	public string MechJebInfoItemsStatsColumn1;

	[NonSerialized]
	public string MechJebInfoItemsStatsColumn2;

	[NonSerialized]
	public string MechJebInfoItemsStatsColumn3;

	[NonSerialized]
	public string MechJebInfoItemsStatsColumn4;

	[NonSerialized]
	public string MechJebInfoItemsStatsColumn5;

	[NonSerialized]
	public string MechJebInfoItemsStatsColumn6;

	[NonSerialized]
	public string MechJebInfoItemsStatsColumn7;

	[NonSerialized]
	public string MechJebInfoItemsStatsColumn8;

	[NonSerialized]
	public string MechJebInfoItemsStatsColumn9;

	[NonSerialized]
	public string MechJebInfoItemsStatsColumn10;

	[NonSerialized]
	public string MechJebInfoItemsStatsColumn11;

	[NonSerialized]
	public string MechJebInfoItemsStatsColumn12;

	[NonSerialized]
	public string MechJebInfoItemsStatsColumn13;

	[NonSerialized]
	public string MechJebAscentTitle;

	[NonSerialized]
	public string MechJebWindowEdTitle;

	[NonSerialized]
	public string MechJebWindowEdCustomInfoWindowLabel1;

	public static CachedLocalizer Instance { get; private set; }

	public static void Bootstrap()
	{
		//IL_0013: Unknown result type (might be due to invalid IL or missing references)
		if (!((Object)(object)Instance != (Object)null))
		{
			Instance = new GameObject("MechJeb.CachedLocalizer").AddComponent<CachedLocalizer>();
		}
	}

	public void OnDestroy()
	{
		Instance = null;
	}

	public void Awake()
	{
		UpdateCachedStrings();
	}

	private void UpdateCachedStrings()
	{
		MechJebAscentButton1 = Localizer.Format("#MechJeb_Ascent_button1");
		MechJebAscentButton2 = Localizer.Format("#MechJeb_Ascent_button2");
		MechJebAscentButton3 = Localizer.Format("#MechJeb_Ascent_button3");
		MechJebAscentButton4 = Localizer.Format("#MechJeb_Ascent_button4");
		MechJebAscentButton5 = Localizer.Format("#MechJeb_Ascent_button5");
		MechJebAscentButton6 = Localizer.Format("#MechJeb_Ascent_button6");
		MechJebAscentButton7 = Localizer.Format("#MechJeb_Ascent_button7");
		MechJebAscentButton8 = Localizer.Format("#MechJeb_Ascent_button8");
		MechJebAscentButton9 = Localizer.Format("#MechJeb_Ascent_button9");
		MechJebAscentButton10 = Localizer.Format("#MechJeb_Ascent_button10");
		MechJebAscentButton11 = Localizer.Format("#MechJeb_Ascent_button11");
		MechJebAscentButton12 = Localizer.Format("#MechJeb_Ascent_button12");
		MechJebAscentButton13 = Localizer.Format("#MechJeb_Ascent_button13");
		MechJebAscentButton15 = Localizer.Format("#MechJeb_Ascent_button15");
		MechJebAscentButton17 = Localizer.Format("#MechJeb_Ascent_button17");
		MechJebAscentLabel1 = Localizer.Format("#MechJeb_Ascent_label1");
		MechJebAscentLabel2 = Localizer.Format("#MechJeb_Ascent_label2");
		MechJebAscentLabel3 = Localizer.Format("#MechJeb_Ascent_label3");
		MechJebAscentLabel4 = Localizer.Format("#MechJeb_Ascent_label4");
		MechJebAscentLabel5 = Localizer.Format("#MechJeb_Ascent_label5");
		MechJebAscentLabel6 = Localizer.Format("#MechJeb_Ascent_label6");
		MechJebAscentLabel8 = Localizer.Format("#MechJeb_Ascent_label8");
		MechJebAscentLabel9 = Localizer.Format("#MechJeb_Ascent_label9");
		MechJebAscentLabel10 = Localizer.Format("#MechJeb_Ascent_label10");
		MechJebAscentLabel11 = Localizer.Format("#MechJeb_Ascent_label11");
		MechJebAscentLabel12 = Localizer.Format("#MechJeb_Ascent_label12");
		MechJebAscentLabel13 = Localizer.Format("#MechJeb_Ascent_label13");
		MechJebAscentLabel14 = Localizer.Format("#MechJeb_Ascent_label14");
		MechJebAscentLabel15 = Localizer.Format("#MechJeb_Ascent_label15");
		MechJebAscentLabel16 = Localizer.Format("#MechJeb_Ascent_label16");
		MechJebAscentLabel17 = Localizer.Format("#MechJeb_Ascent_label17");
		MechJebAscentLabel18 = Localizer.Format("#MechJeb_Ascent_label18");
		MechJebAscentLabel19 = Localizer.Format("#MechJeb_Ascent_label19");
		MechJebAscentLabel20 = Localizer.Format("#MechJeb_Ascent_label20");
		MechJebAscentLabel21 = Localizer.Format("#MechJeb_Ascent_label21");
		MechJebAscentLabel22 = Localizer.Format("#MechJeb_Ascent_label22");
		MechJebAscentLabel23 = Localizer.Format("#MechJeb_Ascent_label23");
		MechJebAscentLabel24 = Localizer.Format("#MechJeb_Ascent_label24");
		MechJebAscentLabel25 = Localizer.Format("#MechJeb_Ascent_label25");
		MechJebAscentLabel26 = Localizer.Format("#MechJeb_Ascent_label26");
		MechJebAscentLabel27 = Localizer.Format("#MechJeb_Ascent_label27");
		MechJebAscentLabel28 = Localizer.Format("#MechJeb_Ascent_label28");
		MechJebAscentLabel29 = Localizer.Format("#MechJeb_Ascent_label29");
		MechJebAscentLabel30 = Localizer.Format("#MechJeb_Ascent_label30");
		MechJebAscentLabel31 = Localizer.Format("#MechJeb_Ascent_label31");
		MechJebAscentLabel32 = Localizer.Format("#MechJeb_Ascent_label32");
		MechJebAscentLabel33 = Localizer.Format("#MechJeb_Ascent_label33");
		MechJebAscentLabel34 = Localizer.Format("#MechJeb_Ascent_label34");
		MechJebAscentLabel35 = Localizer.Format("#MechJeb_Ascent_label35");
		MechJebAscentLabel36 = Localizer.Format("#MechJeb_Ascent_label36");
		MechJebAscentLabel37 = Localizer.Format("#MechJeb_Ascent_label37");
		MechJebAscentLabel38 = Localizer.Format("#MechJeb_Ascent_label38");
		MechJebAscentLabel39 = Localizer.Format("#MechJeb_Ascent_label39");
		MechJebAscentLabel40 = Localizer.Format("#MechJeb_Ascent_label40");
		MechJebAscentLabel41 = Localizer.Format("#MechJeb_Ascent_label41");
		MechJebAscentLabel42 = Localizer.Format("#MechJeb_Ascent_label42");
		MechJebAscentLabel44 = Localizer.Format("#MechJeb_Ascent_label44");
		MechJebAscentCheckbox2 = Localizer.Format("#MechJeb_Ascent_checkbox2");
		MechJebAscentCheckbox3 = Localizer.Format("#MechJeb_Ascent_checkbox3");
		MechJebAscentCheckbox4 = Localizer.Format("#MechJeb_Ascent_checkbox4");
		MechJebAscentCheckbox5 = Localizer.Format("#MechJeb_Ascent_checkbox5");
		MechJebAscentCheckbox6 = Localizer.Format("#MechJeb_Ascent_checkbox6");
		MechJebAscentCheckbox7 = Localizer.Format("#MechJeb_Ascent_checkbox7");
		MechJebAscentCheckbox8 = Localizer.Format("#MechJeb_Ascent_checkbox8");
		MechJebAscentCheckbox9 = Localizer.Format("#MechJeb_Ascent_checkbox9");
		MechJebAscentCheckbox10 = Localizer.Format("#MechJeb_Ascent_checkbox10");
		MechJebAscentCheckbox11 = Localizer.Format("#MechJeb_Ascent_checkbox11");
		MechJebAscentCheckbox12 = Localizer.Format("#MechJeb_Ascent_checkbox12");
		MechJebAscentCheckbox13 = Localizer.Format("#MechJeb_Ascent_checkbox13");
		MechJebAscentCheckbox14 = Localizer.Format("#MechJeb_Ascent_checkbox14");
		MechJebAscentCheckbox15 = Localizer.Format("#MechJeb_Ascent_checkbox15");
		MechJebAscentCheckbox16 = Localizer.Format("#MechJeb_Ascent_checkbox16");
		MechJebAscentCheckbox17 = Localizer.Format("#MechJeb_Ascent_checkbox17");
		MechJebAscentCheckbox18 = Localizer.Format("#MechJeb_Ascent_checkbox18");
		MechJebAscentCheckbox19 = Localizer.Format("#MechJeb_Ascent_checkbox19");
		MechJebAscentCheckbox20 = Localizer.Format("#MechJeb_Ascent_checkbox20");
		MechJebAscentStatus9 = Localizer.Format("#MechJeb_Ascent_status9");
		MechJebAscentStatus10 = Localizer.Format("#MechJeb_Ascent_status10");
		MechJebAscentStatus11 = Localizer.Format("#MechJeb_Ascent_status11");
		MechJebAscentAttachAlt = Localizer.Format("#MechJeb_Ascent_attachAlt");
		MechJebAscentWarnAttachAltHigh = Localizer.Format("#MechJeb_Ascent_warnAttachAltHigh");
		MechJebAscentWarnAttachAltLow = Localizer.Format("#MechJeb_Ascent_warnAttachAltLow");
		MechJebAscentLaunchToTargetLan = Localizer.Format("#MechJeb_Ascent_LaunchToTargetLan");
		MechJebAscentWarnInvalidTarget = Localizer.Format("#MechJeb_Ascent_warnInvalidTarget");
		MechJebAscentLaunchToLan = Localizer.Format("#MechJeb_Ascent_LaunchToLan");
		MechJebAscentLaunchingToTargetLAN = Localizer.Format("#MechJeb_Ascent_LaunchingToTargetLAN");
		MechJebAscentLaunchingToManualLAN = Localizer.Format("#MechJeb_Ascent_LaunchingToManualLAN");
		MechJebAscentMsg2 = Localizer.Format("#MechJeb_Ascent_msg2");
		MechJebAscentHotStaging = Localizer.Format("#MechJeb_Ascent_hotStaging");
		MechJebAscentDropSolids = Localizer.Format("#MechJeb_Ascent_dropSolids");
		MechJebAscentLeadTime = Localizer.Format("#MechJeb_Ascent_leadTime");
		MechJebInfoItemsUnlimitedText = Localizer.Format("#MechJeb_InfoItems_UnlimitedText");
		MechJebInfoItemsLabel1 = Localizer.Format("#MechJeb_InfoItems_label1");
		MechJebInfoItemsButton5 = Localizer.Format("#MechJeb_InfoItems_button5");
		MechJebInfoItemsButton6 = Localizer.Format("#MechJeb_InfoItems_button6");
		MechJebInfoItemsStatsColumn0 = Localizer.Format("#MechJeb_InfoItems_StatsColumn0");
		MechJebInfoItemsStatsColumn1 = Localizer.Format("#MechJeb_InfoItems_StatsColumn1");
		MechJebInfoItemsStatsColumn2 = Localizer.Format("#MechJeb_InfoItems_StatsColumn2");
		MechJebInfoItemsStatsColumn3 = Localizer.Format("#MechJeb_InfoItems_StatsColumn3");
		MechJebInfoItemsStatsColumn4 = Localizer.Format("#MechJeb_InfoItems_StatsColumn4");
		MechJebInfoItemsStatsColumn5 = Localizer.Format("#MechJeb_InfoItems_StatsColumn5");
		MechJebInfoItemsStatsColumn6 = Localizer.Format("#MechJeb_InfoItems_StatsColumn6");
		MechJebInfoItemsStatsColumn7 = Localizer.Format("#MechJeb_InfoItems_StatsColumn7");
		MechJebInfoItemsStatsColumn8 = Localizer.Format("#MechJeb_InfoItems_StatsColumn8");
		MechJebInfoItemsStatsColumn9 = Localizer.Format("#MechJeb_InfoItems_StatsColumn9");
		MechJebInfoItemsStatsColumn10 = Localizer.Format("#MechJeb_InfoItems_StatsColumn10");
		MechJebInfoItemsStatsColumn11 = Localizer.Format("#MechJeb_InfoItems_StatsColumn11");
		MechJebInfoItemsStatsColumn12 = Localizer.Format("#MechJeb_InfoItems_StatsColumn12");
		MechJebInfoItemsStatsColumn13 = Localizer.Format("#MechJeb_InfoItems_StatsColumn13");
		MechJebInfoItemsShowEmpty = Localizer.Format("#MechJeb_InfoItems_showEmpty");
		MechJebInfoItemsHideEmpty = Localizer.Format("#MechJeb_InfoItems_hideEmpty");
		MechJebNavBallGuidanceBtn1 = Localizer.Format("#MechJeb_NavBallGuidance_btn1");
		MechJebNavBallGuidanceBtn2 = Localizer.Format("#MechJeb_NavBallGuidance_btn2");
		MechJebAscentTitle = Localizer.Format("#MechJeb_Ascent_title");
		MechJebWindowEdTitle = Localizer.Format("#MechJeb_WindowEd_title");
		MechJebWindowEdCustomInfoWindowLabel1 = Localizer.Format("#MechJeb_WindowEd_CustomInfoWindow_Label1");
	}
}
