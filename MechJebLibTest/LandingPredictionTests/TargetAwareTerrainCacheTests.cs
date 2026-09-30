using System;
using MuMech.Landing;
using Xunit;

namespace MechJebLibTest.LandingPredictionTests
{
    public class TargetAwareTerrainCacheTests
    {
        [Fact]
        public void ReusesOnlyExactlyTheSameBodyFixedCoordinates()
        {
            var cache = new TargetAwareTerrainCache();
            cache.Store(1, 23, 492);
            Assert.True(cache.TryGet(1, 23, out double height));
            Assert.Equal(492, height);
            // Even a nearby location must query PQS: no smoothing over ridges.
            Assert.False(cache.TryGet(1, 23.000000001, out _));
            Assert.False(cache.TryGet(1.000000001, 23, out _));
        }

        [Fact]
        public void RepeatedContextRetainsSamplesButBodyOrPqsChangeClearsThem()
        {
            var cache = new TargetAwareTerrainCache();
            var body = new object();
            var pqs = new object();
            cache.Configure(body, pqs, -400, 8300, 492);
            cache.Store(1, 23, 492);
            cache.Configure(body, pqs, -400, 8300, 492);
            Assert.Equal(1, cache.Count);
            cache.Configure(new object(), pqs, -400, 8300, 492);
            Assert.Equal(0, cache.Count);
            cache.Store(1, 23, 492);
            cache.Configure(body, new object(), -400, 8300, 492);
            Assert.Equal(0, cache.Count);
        }

        [Theory]
        [InlineData(-401, 8300, 492)]
        [InlineData(-400, 8301, 492)]
        public void TerrainDescriptorChangeInvalidatesReuse(double min, double max, double target)
        {
            var cache = new TargetAwareTerrainCache();
            var body = new object();
            var pqs = new object();
            cache.Configure(body, pqs, -400, 8300, 492);
            cache.Store(1, 23, 492);
            cache.Configure(body, pqs, min, max, target);
            Assert.Equal(0, cache.Count);
        }

        [Fact]
        public void AdvancingQueriesShareLocationsWithoutPreloadingAndTargetChangeRetainsTerrain()
        {
            var cache = new TargetAwareTerrainCache();
            var body = new object(); var pqs = new object();
            cache.Configure(body, pqs, -400, 8300, 492);
            TargetAwareTerrainCache.QueryLocation(0.67416, 23.47314, 200000, 1, out double lat, out double lon);
            TargetAwareTerrainCache.QueryLocation(0.674160001, 23.473140001, 200000, 1,
                out double nextLat, out double nextLon);
            Assert.Equal(lat, nextLat); Assert.Equal(lon, nextLon);
            Assert.Equal(0, cache.Count);
            cache.Store(lat, lon, 492);
            cache.Configure(body, pqs, -400, 8300, 493);
            Assert.True(cache.TryGet(nextLat, nextLon, out double height));
            Assert.Equal(492, height); Assert.Equal(1, cache.Count);
        }

        [Fact]
        public void FineSamplingDistinguishesRidgeWithinCoarseCellAndWrapsLongitude()
        {
            TargetAwareTerrainCache.QueryLocation(0, 23, 200000, 25, out double coarseLat, out double coarseLon);
            TargetAwareTerrainCache.QueryLocation(0, coarseLon + 0.001, 200000, 25,
                out double nextCoarseLat, out double nextCoarseLon);
            Assert.Equal(coarseLat, nextCoarseLat); Assert.Equal(coarseLon, nextCoarseLon);
            TargetAwareTerrainCache.QueryLocation(0, coarseLon, 200000, 1, out double fineLat, out double fineLon);
            TargetAwareTerrainCache.QueryLocation(0, coarseLon + 0.001, 200000, 1,
                out double ridgeLat, out double ridgeLon);
            var cache = new TargetAwareTerrainCache();
            cache.Store(fineLat, fineLon, 10); cache.Store(ridgeLat, ridgeLon, 100);
            Assert.True(cache.TryGet(fineLat, fineLon, out double low));
            Assert.True(cache.TryGet(ridgeLat, ridgeLon, out double high));
            Assert.Equal(10, low); Assert.Equal(100, high);
            TargetAwareTerrainCache.QueryLocation(0, 383, 200000, 25, out _, out double wrapped);
            Assert.Equal(coarseLon, wrapped);
        }

        [Fact]
        public void StorageIsBoundedAndClearRemovesAllSamples()
        {
            var cache = new TargetAwareTerrainCache(2);
            cache.Store(1, 1, 10);
            cache.Store(1, 2, 20);
            cache.Store(1, 2, 20);
            cache.Store(1, 3, 30);
            Assert.Equal(2, cache.Count);
            Assert.False(cache.TryGet(1, 1, out _));
            Assert.True(cache.TryGet(1, 2, out _));
            cache.Clear();
            Assert.Equal(0, cache.Count);
        }

        [Fact]
        public void UnresolvedSamplesAreNeverCached()
        {
            var cache = new TargetAwareTerrainCache();
            Assert.Throws<ArgumentException>(() => cache.Store(1, 23, double.NaN));
            Assert.Throws<ArgumentException>(() => cache.Store(1, double.PositiveInfinity, 492));
            Assert.Equal(0, cache.Count);
        }
    }
}
