# Bench2Drive (220 routes) — what each episode tests

Bench2Drive has NO explicit easy/medium/hard tiers. Difficulty comes from four things that vary per route:
1. the **scenario type** (44 types, 5 routes each = 220), grouped by the benchmark into **5 "abilities"** used for the multi-ability score;
2. the **town** (Town01-07, 10 are small/urban; Town11/12/13/15 are large maps with highways);
3. **weather / time of day** (day, night, rain, fog combinations);
4. **route length** (most are short, ~100-400 m, built around the single scenario).
Scoring: Driving Score = route completion x infraction penalty (collisions, red lights, stop signs, off-road, blocked, timeout, "min speed").
Success = DS 100 (no infraction) is what the Success Rate counts. The official subsets: full 220, and the 10-route "dev10" smoke set (marked *).

## The 5 abilities (official grouping, from Bench2Drive/tools/ability_benchmark.py)

- **Overtaking** (45 routes): Accident, AccidentTwoWays, ConstructionObstacle, ConstructionObstacleTwoWays, HazardAtSideLaneTwoWays, HazardAtSideLane, ParkedObstacleTwoWays, ParkedObstacle, VehicleOpensDoorTwoWays
- **Merging** (80 routes): CrossingBicycleFlow, EnterActorFlow, HighwayExit, InterurbanActorFlow, HighwayCutIn, InterurbanAdvancedActorFlow, MergerIntoSlowTrafficV2, MergerIntoSlowTraffic, NonSignalizedJunctionLeftTurn, NonSignalizedJunctionRightTurn, NonSignalizedJunctionLeftTurnEnterFlow, ParkingExit, SequentialLaneChange, SignalizedJunctionLeftTurn, SignalizedJunctionRightTurn, SignalizedJunctionLeftTurnEnterFlow
- **Emergency_Brake** (60 routes): BlockedIntersection, DynamicObjectCrossing, HardBreakRoute, OppositeVehicleTakingPriority, OppositeVehicleRunningRedLight, ParkingCutIn, PedestrianCrossing, ParkingCrossingPedestrian, StaticCutIn, VehicleTurningRoute, VehicleTurningRoutePedestrian, ControlLoss
- **Give_Way** (10 routes): InvadingTurn, YieldToEmergencyVehicle
- **Traffic_Signs** (95 routes): BlockedIntersection, OppositeVehicleTakingPriority, OppositeVehicleRunningRedLight, PedestrianCrossing, VehicleTurningRoute, VehicleTurningRoutePedestrian, EnterActorFlow, CrossingBicycleFlow, NonSignalizedJunctionLeftTurn, NonSignalizedJunctionRightTurn, NonSignalizedJunctionLeftTurnEnterFlow, SignalizedJunctionLeftTurn, SignalizedJunctionRightTurn, SignalizedJunctionLeftTurnEnterFlow, T_Junction, VanillaNonSignalizedTurn, VanillaSignalizedTurnEncounterGreenLight, VanillaSignalizedTurnEncounterRedLight, VanillaNonSignalizedTurnEncounterStopsign

## The 44 scenario types (5 routes each)

| scenario | ability | what happens | routes |
|---|---|---|---|
| Accident | Overtaking | crashed cars block your lane; overtake via the other lane | 2534, 2539, 3307, 24240, 24816 |
| AccidentTwoWays | Overtaking | same, but the only way round is the opposite-direction lane | 1852, 2643, 3410, 25845, 25857 |
| BlockedIntersection | Emergency_Brake/Traffic_Signs | a vehicle blocks the junction you must enter | 2204, 2215, 2989, 27494*, 27532 |
| ConstructionObstacle | Overtaking | construction zone (cones/barriers) blocks the lane | 2509, 2513, 24367, 24785, 24795 |
| ConstructionObstacleTwoWays | Overtaking | construction zone, detour through oncoming lane | 1825, 1833, 2606, 20920, 25424* |
| ControlLoss | Emergency_Brake | the car loses traction (control noise) - keep the lane | 3561, 24340, 24784, 24841, 26418 |
| CrossingBicycleFlow | Merging/Traffic_Signs | cross a stream of bicycles at a junction | 3086, 3090, 3093, 3099, 3100 |
| DynamicObjectCrossing | Emergency_Brake | pedestrian/object suddenly crosses from behind an occluder | 17752, 24211, 24224, 24252, 24333 |
| EnterActorFlow | Merging/Traffic_Signs | merge into a dense flow of vehicles | 2201, 3749, 11715, 11755, 27506 |
| HardBreakRoute | Emergency_Brake | the car ahead brakes hard | 3540, 24330, 24781, 26406, 26456 |
| HazardAtSideLane | Overtaking | slow cyclists/vehicles in the side lane to pass | 1790, 1792, 25300, 25381*, 25439 |
| HazardAtSideLaneTwoWays | Overtaking | same with oncoming traffic in the passing lane | 3436, 24258, 25854, 25951, 25955 |
| HighwayCutIn | Merging | vehicle cuts in from the on-ramp on a highway | 2286, 3072, 3074, 3080, 3813 |
| HighwayExit | Merging | take a highway exit through traffic | 23658, 23659, 23670, 23687, 24041 |
| InterurbanActorFlow | Merging | merge onto an interurban road with traffic | 23901, 23910, 23918, 23930, 24098 |
| InterurbanAdvancedActorFlow | Merging | merge across a fast interurban flow | 23695, 23700, 23708, 24071, 24078 |
| InvadingTurn | Give_Way | oncoming vehicle invades your lane in a bend | 2790, 2802, 3564, 3572, 3575 |
| MergerIntoSlowTraffic | Merging | merge into slow-moving traffic | 2273, 2283, 3048, 3785, 3800 |
| MergerIntoSlowTrafficV2 | Merging | merge into slow traffic (variant) | 23771, 24092, 26394, 26401, 26408 |
| NonSignalizedJunctionLeftTurn | Merging/Traffic_Signs | unsignalised left turn across traffic | 2084, 2086, 2091*, 2881, 26990 |
| NonSignalizedJunctionLeftTurnEnterFlow | Merging/Traffic_Signs | unsignalised left turn into a flow | 28087, 28093, 28210, 28219, 28229 |
| NonSignalizedJunctionRightTurn | Merging/Traffic_Signs | unsignalised right turn into traffic | 2115, 2903, 3666, 3670, 3676 |
| OppositeVehicleRunningRedLight | Emergency_Brake/Traffic_Signs | a car runs the red light across your path | 2082, 2844, 2847, 26944, 26950 |
| OppositeVehicleTakingPriority | Emergency_Brake/Traffic_Signs | oncoming car ignores priority at a junction | 2127, 2129, 2143, 2913, 3697 |
| ParkedObstacle | Overtaking | parked vehicle sticking into your lane | 1773, 2554, 24757, 25318, 25383 |
| ParkedObstacleTwoWays | Overtaking | same, detour via oncoming lane | 2664, 2668, 3457, 25865, 25896 |
| ParkingCrossingPedestrian | Emergency_Brake | pedestrian steps out between parked cars | 3248, 3255*, 18252, 24206, 24294 |
| ParkingCutIn | Emergency_Brake | a parked car pulls out right in front of you | 1711, 18305, 18311, 18356, 24759 |
| ParkingExit | Merging | leave a parking spot into traffic | 1956, 3514*, 3520, 26393, 26435 |
| PedestrianCrossing | Emergency_Brake/Traffic_Signs | pedestrians cross at the junction | 14194, 25863, 27515, 27529, 27582 |
| SequentialLaneChange | Merging | several lane changes in a row through traffic | 17563, 17569*, 17598, 17635, 17655 |
| SignalizedJunctionLeftTurn | Merging/Traffic_Signs | left turn at a traffic light across oncoming cars | 3936, 4183, 4468, 4669, 4683 |
| SignalizedJunctionLeftTurnEnterFlow | Merging/Traffic_Signs | left turn at a light into a flow | 28099, 28198*, 28241, 28243, 28330 |
| SignalizedJunctionRightTurn | Merging/Traffic_Signs | right turn at a light into cross traffic | 2050, 4937, 26956, 26966, 27018 |
| StaticCutIn | Emergency_Brake | a vehicle from the adjacent lane cuts in | 2709, 2715, 25358, 26396, 26405* |
| T_Junction | Traffic_Signs | navigate a T-junction with traffic | 26458, 28035, 28048, 28111, 28154 |
| VanillaNonSignalizedTurn | Traffic_Signs | plain unsignalised turn | 2390, 2397, 2403, 3178, 3890 |
| VanillaNonSignalizedTurnEncounterStopsign | Traffic_Signs | turn at a stop sign (stop) | 2416, 3184, 3189, 3904, 3905 |
| VanillaSignalizedTurnEncounterGreenLight | Traffic_Signs | turn at a green light | 14842, 14862, 14909, 25968, 25975 |
| VanillaSignalizedTurnEncounterRedLight | Traffic_Signs | turn at a red light (wait) | 2373, 3144, 3865, 3869, 3876 |
| VehicleOpensDoorTwoWays | Overtaking | parked car opens its door into your path | 3464, 3472, 3476, 3482, 25928 |
| VehicleTurningRoute | Emergency_Brake/Traffic_Signs | turning vehicle/bike appears in your turn path | 2144, 2943, 3708, 3712, 3717 |
| VehicleTurningRoutePedestrian | Emergency_Brake/Traffic_Signs | pedestrian appears in your turn path | 2164, 3731, 3737, 10857, 11381 |
| YieldToEmergencyVehicle | Give_Way | emergency vehicle behind: pull aside | 3364, 3373, 3378, 3380, 25378* |

## All 220 routes (* = in dev10)

| route | town | scenario | ability | weather | length |
|---|---|---|---|---|---|
| 24841 | Town01 | ControlLoss | Emergency_Brake | day, fog 2% | 132 m |
| 24211 | Town01 | DynamicObjectCrossing | Emergency_Brake | day, rain 50%, fog 10% | 134 m |
| 24781 | Town01 | HardBreakRoute | Emergency_Brake | day, rain 100%, fog 7% | 130 m |
| 28035 | Town01 | T_Junction | Traffic_Signs | day, fog 2% | 113 m |
| 24784 | Town02 | ControlLoss | Emergency_Brake | day, fog 50% | 104 m |
| 24224 | Town02 | DynamicObjectCrossing | Emergency_Brake | day, fog 3% | 134 m |
| 24294 | Town02 | ParkingCrossingPedestrian | Emergency_Brake | day, fog 10% | 104 m |
| 28048 | Town02 | T_Junction | Traffic_Signs | day, rain 50%, fog 10% | 78 m |
| 24816 | Town03 | Accident | Overtaking | night, rain 100%, fog 100% | 126 m |
| 26456 | Town03 | HardBreakRoute | Emergency_Brake | day, fog 10% | 132 m |
| 25854 | Town03 | HazardAtSideLaneTwoWays | Overtaking | day, fog 2% | 132 m |
| 26990 | Town03 | NonSignalizedJunctionLeftTurn | Merging/Traffic_Signs | day, rain 60%, fog 50% | 79 m |
| 26950 | Town03 | OppositeVehicleRunningRedLight | Emergency_Brake/Traffic_Signs | night, fog 2% | 73 m |
| 25896 | Town03 | ParkedObstacleTwoWays | Overtaking | night, rain 60%, fog 60% | 132 m |
| 24206 | Town03 | ParkingCrossingPedestrian | Emergency_Brake | day, rain 100%, fog 7% | 134 m |
| 26393 | Town03 | ParkingExit | Merging | day, fog 50% | 136 m |
| 26435 | Town03 | ParkingExit | Merging | day, rain 100%, fog 7% | 132 m |
| 27515 | Town03 | PedestrianCrossing | Emergency_Brake/Traffic_Signs | day, fog 2% | 77 m |
| 25378* | Town03 | YieldToEmergencyVehicle | Give_Way | day, rain 60%, fog 50% | 96 m |
| 27494* | Town04 | BlockedIntersection | Emergency_Brake/Traffic_Signs | day, rain 60%, fog 50% | 82 m |
| 24785 | Town04 | ConstructionObstacle | Overtaking | day, rain 100%, fog 10% | 146 m |
| 24795 | Town04 | ConstructionObstacle | Overtaking | day, fog 2% | 143 m |
| 26406 | Town04 | HardBreakRoute | Emergency_Brake | day, fog 2% | 146 m |
| 25300 | Town04 | HazardAtSideLane | Overtaking | day | 156 m |
| 25951 | Town04 | HazardAtSideLaneTwoWays | Overtaking | day, fog 2% | 136 m |
| 28087 | Town04 | NonSignalizedJunctionLeftTurnEnterFlow | Merging/Traffic_Signs | day, rain 100%, fog 7% | 75 m |
| 28093 | Town04 | NonSignalizedJunctionLeftTurnEnterFlow | Merging/Traffic_Signs | day, rain 100%, fog 10% | 77 m |
| 26944 | Town04 | OppositeVehicleRunningRedLight | Emergency_Brake/Traffic_Signs | day, rain 60%, fog 50% | 88 m |
| 27529 | Town04 | PedestrianCrossing | Emergency_Brake/Traffic_Signs | night, rain 60%, fog 60% | 66 m |
| 28099 | Town04 | SignalizedJunctionLeftTurnEnterFlow | Merging/Traffic_Signs | night, fog 60% | 80 m |
| 28111 | Town04 | T_Junction | Traffic_Signs | day, rain 60%, fog 50% | 87 m |
| 25857 | Town05 | AccidentTwoWays | Overtaking | day, fog 3% | 131 m |
| 27506 | Town05 | EnterActorFlow | Merging/Traffic_Signs | night, rain 60%, fog 60% | 75 m |
| 25381* | Town05 | HazardAtSideLane | Overtaking | day, fog 50% | 135 m |
| 25955 | Town05 | HazardAtSideLaneTwoWays | Overtaking | day, rain 60%, fog 50% | 136 m |
| 25318 | Town05 | ParkedObstacle | Overtaking | night, rain 30%, fog 3% | 143 m |
| 24759 | Town05 | ParkingCutIn | Emergency_Brake | day, rain 60%, fog 50% | 137 m |
| 26956 | Town05 | SignalizedJunctionRightTurn | Merging/Traffic_Signs | day | 70 m |
| 26966 | Town05 | SignalizedJunctionRightTurn | Merging/Traffic_Signs | day, rain 100%, fog 7% | 74 m |
| 26396 | Town05 | StaticCutIn | Emergency_Brake | day, rain 50%, fog 10% | 136 m |
| 24367 | Town06 | ConstructionObstacle | Overtaking | day, rain 100%, fog 7% | 134 m |
| 26394 | Town06 | MergerIntoSlowTrafficV2 | Merging | day, fog 50% | 137 m |
| 26401 | Town06 | MergerIntoSlowTrafficV2 | Merging | night, fog 60% | 124 m |
| 26408 | Town06 | MergerIntoSlowTrafficV2 | Merging | day, rain 60%, fog 3% | 139 m |
| 24757 | Town06 | ParkedObstacle | Overtaking | day, rain 60%, fog 50% | 134 m |
| 25358 | Town06 | StaticCutIn | Emergency_Brake | day, fog 50% | 139 m |
| 24258 | Town07 | HazardAtSideLaneTwoWays | Overtaking | day, fog 2% | 132 m |
| 25863 | Town07 | PedestrianCrossing | Emergency_Brake/Traffic_Signs | day, rain 60%, fog 50% | 86 m |
| 26458 | Town07 | T_Junction | Traffic_Signs | day, rain 100%, fog 7% | 73 m |
| 25968 | Town07 | VanillaSignalizedTurnEncounterGreenLight | Traffic_Signs | day, fog 2% | 51 m |
| 25975 | Town07 | VanillaSignalizedTurnEncounterGreenLight | Traffic_Signs | day, rain 100%, fog 7% | 49 m |
| 24240 | Town10HD | Accident | Overtaking | night, rain 60%, fog 60% | 102 m |
| 24340 | Town10HD | ControlLoss | Emergency_Brake | day, fog 10% | 102 m |
| 24330 | Town10HD | HardBreakRoute | Emergency_Brake | night, rain 100%, fog 3% | 102 m |
| 28154 | Town10HD | T_Junction | Traffic_Signs | day, fog 10% | 155 m |
| 27532 | Town11 | BlockedIntersection | Emergency_Brake/Traffic_Signs | day, fog 2% | 67 m |
| 25424* | Town11 | ConstructionObstacleTwoWays | Overtaking | day, rain 60%, fog 50% | 108 m |
| 26418 | Town11 | ControlLoss | Emergency_Brake | day, rain 100%, fog 10% | 134 m |
| 24252 | Town11 | DynamicObjectCrossing | Emergency_Brake | day, rain 100%, fog 7% | 135 m |
| 25865 | Town11 | ParkedObstacleTwoWays | Overtaking | day, fog 50% | 102 m |
| 27582 | Town11 | PedestrianCrossing | Emergency_Brake/Traffic_Signs | day, fog 3% | 65 m |
| 25928 | Town11 | VehicleOpensDoorTwoWays | Overtaking | day, fog 2% | 102 m |
| 2534 | Town12 | Accident | Overtaking | day, fog 10% | 134 m |
| 2539 | Town12 | Accident | Overtaking | day, rain 60%, fog 50% | 134 m |
| 1852 | Town12 | AccidentTwoWays | Overtaking | day, fog 2% | 134 m |
| 2643 | Town12 | AccidentTwoWays | Overtaking | day, rain 60%, fog 50% | 104 m |
| 2204 | Town12 | BlockedIntersection | Emergency_Brake/Traffic_Signs | day, rain 100%, fog 10% | 70 m |
| 2215 | Town12 | BlockedIntersection | Emergency_Brake/Traffic_Signs | night, rain 100%, fog 100% | 69 m |
| 2989 | Town12 | BlockedIntersection | Emergency_Brake/Traffic_Signs | night, fog 2% | 71 m |
| 2509 | Town12 | ConstructionObstacle | Overtaking | day, fog 2% | 135 m |
| 2513 | Town12 | ConstructionObstacle | Overtaking | day, rain 60%, fog 50% | 134 m |
| 1825 | Town12 | ConstructionObstacleTwoWays | Overtaking | night, rain 100%, fog 100% | 134 m |
| 1833 | Town12 | ConstructionObstacleTwoWays | Overtaking | day, fog 2% | 134 m |
| 2606 | Town12 | ConstructionObstacleTwoWays | Overtaking | day, fog 2% | 134 m |
| 20920 | Town12 | ConstructionObstacleTwoWays | Overtaking | day, rain 50%, fog 10% | 134 m |
| 3086 | Town12 | CrossingBicycleFlow | Merging/Traffic_Signs | day, fog 50% | 79 m |
| 3090 | Town12 | CrossingBicycleFlow | Merging/Traffic_Signs | day, rain 100%, fog 7% | 79 m |
| 3093 | Town12 | CrossingBicycleFlow | Merging/Traffic_Signs | night, fog 2% | 82 m |
| 3099 | Town12 | CrossingBicycleFlow | Merging/Traffic_Signs | night, rain 100%, fog 100% | 80 m |
| 3100 | Town12 | CrossingBicycleFlow | Merging/Traffic_Signs | day, fog 2% | 76 m |
| 17752 | Town12 | DynamicObjectCrossing | Emergency_Brake | night, fog 2% | 134 m |
| 2201 | Town12 | EnterActorFlow | Merging/Traffic_Signs | day, rain 60%, fog 50% | 79 m |
| 11715 | Town12 | EnterActorFlow | Merging/Traffic_Signs | day, fog 2% | 86 m |
| 11755 | Town12 | EnterActorFlow | Merging/Traffic_Signs | day, fog 10% | 82 m |
| 1790 | Town12 | HazardAtSideLane | Overtaking | day, rain 100%, fog 7% | 120 m |
| 1792 | Town12 | HazardAtSideLane | Overtaking | day, fog 2% | 134 m |
| 2286 | Town12 | HighwayCutIn | Merging | day, fog 2% | 106 m |
| 3072 | Town12 | HighwayCutIn | Merging | night, rain 60%, fog 60% | 138 m |
| 3074 | Town12 | HighwayCutIn | Merging | day, fog 2% | 108 m |
| 3080 | Town12 | HighwayCutIn | Merging | day, fog 10% | 137 m |
| 23658 | Town12 | HighwayExit | Merging | day, rain 30%, fog 3% | 106 m |
| 23659 | Town12 | HighwayExit | Merging | day, rain 60%, fog 3% | 100 m |
| 23687 | Town12 | HighwayExit | Merging | day, rain 60%, fog 50% | 154 m |
| 23910 | Town12 | InterurbanActorFlow | Merging | day, fog 2% | 106 m |
| 23930 | Town12 | InterurbanActorFlow | Merging | night, rain 100%, fog 100% | 112 m |
| 24098 | Town12 | InterurbanActorFlow | Merging | day, fog 10% | 106 m |
| 23700 | Town12 | InterurbanAdvancedActorFlow | Merging | night, rain 100%, fog 100% | 113 m |
| 23708 | Town12 | InterurbanAdvancedActorFlow | Merging | day, fog 2% | 148 m |
| 24078 | Town12 | InterurbanAdvancedActorFlow | Merging | day, rain 60%, fog 50% | 113 m |
| 2790 | Town12 | InvadingTurn | Give_Way | day, rain 30%, fog 3% | 100 m |
| 2802 | Town12 | InvadingTurn | Give_Way | day, rain 100%, fog 10% | 114 m |
| 2273 | Town12 | MergerIntoSlowTraffic | Merging | day, fog 3% | 138 m |
| 2283 | Town12 | MergerIntoSlowTraffic | Merging | day, rain 50%, fog 10% | 140 m |
| 3048 | Town12 | MergerIntoSlowTraffic | Merging | day, fog 2% | 164 m |
| 23771 | Town12 | MergerIntoSlowTrafficV2 | Merging | day, fog 2% | 146 m |
| 2084 | Town12 | NonSignalizedJunctionLeftTurn | Merging/Traffic_Signs | night, rain 60%, fog 60% | 74 m |
| 2086 | Town12 | NonSignalizedJunctionLeftTurn | Merging/Traffic_Signs | day, fog 2% | 74 m |
| 2091* | Town12 | NonSignalizedJunctionLeftTurn | Merging/Traffic_Signs | day, fog 3% | 73 m |
| 2881 | Town12 | NonSignalizedJunctionLeftTurn | Merging/Traffic_Signs | day, rain 50%, fog 10% | 74 m |
| 28210 | Town12 | NonSignalizedJunctionLeftTurnEnterFlow | Merging/Traffic_Signs | day, fog 2% | 77 m |
| 28219 | Town12 | NonSignalizedJunctionLeftTurnEnterFlow | Merging/Traffic_Signs | day, fog 2% | 77 m |
| 28229 | Town12 | NonSignalizedJunctionLeftTurnEnterFlow | Merging/Traffic_Signs | day, fog 50% | 83 m |
| 2115 | Town12 | NonSignalizedJunctionRightTurn | Merging/Traffic_Signs | day, rain 60%, fog 3% | 68 m |
| 2903 | Town12 | NonSignalizedJunctionRightTurn | Merging/Traffic_Signs | day, rain 60%, fog 50% | 78 m |
| 2082 | Town12 | OppositeVehicleRunningRedLight | Emergency_Brake/Traffic_Signs | night, fog 60% | 78 m |
| 2844 | Town12 | OppositeVehicleRunningRedLight | Emergency_Brake/Traffic_Signs | day, rain 60%, fog 3% | 78 m |
| 2847 | Town12 | OppositeVehicleRunningRedLight | Emergency_Brake/Traffic_Signs | day, fog 2% | 84 m |
| 2127 | Town12 | OppositeVehicleTakingPriority | Emergency_Brake/Traffic_Signs | day, rain 50%, fog 10% | 73 m |
| 2129 | Town12 | OppositeVehicleTakingPriority | Emergency_Brake/Traffic_Signs | day, rain 60%, fog 50% | 77 m |
| 2143 | Town12 | OppositeVehicleTakingPriority | Emergency_Brake/Traffic_Signs | day, fog 3% | 74 m |
| 2913 | Town12 | OppositeVehicleTakingPriority | Emergency_Brake/Traffic_Signs | night, rain 100%, fog 3% | 75 m |
| 1773 | Town12 | ParkedObstacle | Overtaking | night, rain 100%, fog 100% | 134 m |
| 2554 | Town12 | ParkedObstacle | Overtaking | day, fog 2% | 134 m |
| 2664 | Town12 | ParkedObstacleTwoWays | Overtaking | day, fog 10% | 134 m |
| 2668 | Town12 | ParkedObstacleTwoWays | Overtaking | day, rain 100%, fog 7% | 114 m |
| 18252 | Town12 | ParkingCrossingPedestrian | Emergency_Brake | night, rain 100%, fog 100% | 134 m |
| 1711 | Town12 | ParkingCutIn | Emergency_Brake | day, rain 50%, fog 10% | 134 m |
| 18305 | Town12 | ParkingCutIn | Emergency_Brake | day, fog 2% | 134 m |
| 18311 | Town12 | ParkingCutIn | Emergency_Brake | day, fog 10% | 134 m |
| 18356 | Town12 | ParkingCutIn | Emergency_Brake | night, rain 100%, fog 100% | 134 m |
| 1956 | Town12 | ParkingExit | Merging | day, fog 2% | 134 m |
| 14194 | Town12 | PedestrianCrossing | Emergency_Brake/Traffic_Signs | day, fog 2% | 76 m |
| 17563 | Town12 | SequentialLaneChange | Merging | day, fog 50% | 117 m |
| 17569* | Town12 | SequentialLaneChange | Merging | day, fog 2% | 117 m |
| 17598 | Town12 | SequentialLaneChange | Merging | night, rain 100%, fog 3% | 122 m |
| 17635 | Town12 | SequentialLaneChange | Merging | day, fog 10% | 117 m |
| 17655 | Town12 | SequentialLaneChange | Merging | day, fog 2% | 122 m |
| 3936 | Town12 | SignalizedJunctionLeftTurn | Merging/Traffic_Signs | day, fog 50% | 78 m |
| 4183 | Town12 | SignalizedJunctionLeftTurn | Merging/Traffic_Signs | day, fog 2% | 68 m |
| 4468 | Town12 | SignalizedJunctionLeftTurn | Merging/Traffic_Signs | night, rain 100%, fog 100% | 75 m |
| 4669 | Town12 | SignalizedJunctionLeftTurn | Merging/Traffic_Signs | day, fog 2% | 73 m |
| 4683 | Town12 | SignalizedJunctionLeftTurn | Merging/Traffic_Signs | day, fog 10% | 73 m |
| 28241 | Town12 | SignalizedJunctionLeftTurnEnterFlow | Merging/Traffic_Signs | day, fog 2% | 73 m |
| 28243 | Town12 | SignalizedJunctionLeftTurnEnterFlow | Merging/Traffic_Signs | day, rain 30%, fog 3% | 78 m |
| 28330 | Town12 | SignalizedJunctionLeftTurnEnterFlow | Merging/Traffic_Signs | day, fog 10% | 82 m |
| 2050 | Town12 | SignalizedJunctionRightTurn | Merging/Traffic_Signs | day, rain 100%, fog 7% | 74 m |
| 4937 | Town12 | SignalizedJunctionRightTurn | Merging/Traffic_Signs | day, fog 2% | 62 m |
| 2709 | Town12 | StaticCutIn | Emergency_Brake | night, rain 100%, fog 100% | 135 m |
| 2715 | Town12 | StaticCutIn | Emergency_Brake | day, fog 3% | 134 m |
| 2390 | Town12 | VanillaNonSignalizedTurn | Traffic_Signs | day, fog 2% | 60 m |
| 2397 | Town12 | VanillaNonSignalizedTurn | Traffic_Signs | night, rain 100%, fog 100% | 60 m |
| 2403 | Town12 | VanillaNonSignalizedTurn | Traffic_Signs | day, fog 3% | 62 m |
| 3178 | Town12 | VanillaNonSignalizedTurn | Traffic_Signs | day, fog 2% | 58 m |
| 2416 | Town12 | VanillaNonSignalizedTurnEncounterStopsign | Traffic_Signs | day, fog 2% | 52 m |
| 3184 | Town12 | VanillaNonSignalizedTurnEncounterStopsign | Traffic_Signs | day, fog 10% | 58 m |
| 3189 | Town12 | VanillaNonSignalizedTurnEncounterStopsign | Traffic_Signs | day, rain 60%, fog 50% | 57 m |
| 14842 | Town12 | VanillaSignalizedTurnEncounterGreenLight | Traffic_Signs | night, rain 100%, fog 100% | 54 m |
| 14862 | Town12 | VanillaSignalizedTurnEncounterGreenLight | Traffic_Signs | night, fog 2% | 57 m |
| 14909 | Town12 | VanillaSignalizedTurnEncounterGreenLight | Traffic_Signs | day, rain 100%, fog 10% | 56 m |
| 2373 | Town12 | VanillaSignalizedTurnEncounterRedLight | Traffic_Signs | day, fog 2% | 61 m |
| 3144 | Town12 | VanillaSignalizedTurnEncounterRedLight | Traffic_Signs | day, fog 2% | 51 m |
| 2144 | Town12 | VehicleTurningRoute | Emergency_Brake/Traffic_Signs | day, fog 10% | 71 m |
| 2943 | Town12 | VehicleTurningRoute | Emergency_Brake/Traffic_Signs | night, rain 100%, fog 100% | 71 m |
| 2164 | Town12 | VehicleTurningRoutePedestrian | Emergency_Brake/Traffic_Signs | day, fog 2% | 70 m |
| 10857 | Town12 | VehicleTurningRoutePedestrian | Emergency_Brake/Traffic_Signs | day, fog 2% | 73 m |
| 11381 | Town12 | VehicleTurningRoutePedestrian | Emergency_Brake/Traffic_Signs | night, fog 60% | 73 m |
| 3307 | Town13 | Accident | Overtaking | day, fog 2% | 134 m |
| 3410 | Town13 | AccidentTwoWays | Overtaking | night, rain 100%, fog 100% | 133 m |
| 3561 | Town13 | ControlLoss | Emergency_Brake | night, rain 30%, fog 3% | 133 m |
| 3749 | Town13 | EnterActorFlow | Merging/Traffic_Signs | day, fog 2% | 82 m |
| 3540 | Town13 | HardBreakRoute | Emergency_Brake | night, rain 100%, fog 100% | 136 m |
| 3436 | Town13 | HazardAtSideLaneTwoWays | Overtaking | night, rain 100%, fog 100% | 134 m |
| 3813 | Town13 | HighwayCutIn | Merging | day, fog 50% | 142 m |
| 23670 | Town13 | HighwayExit | Merging | day, rain 50%, fog 10% | 166 m |
| 24041 | Town13 | HighwayExit | Merging | night, rain 30%, fog 3% | 161 m |
| 23901 | Town13 | InterurbanActorFlow | Merging | day, fog 2% | 159 m |
| 23918 | Town13 | InterurbanActorFlow | Merging | day, rain 100%, fog 7% | 134 m |
| 23695 | Town13 | InterurbanAdvancedActorFlow | Merging | night, fog 2% | 154 m |
| 24071 | Town13 | InterurbanAdvancedActorFlow | Merging | day, fog 2% | 154 m |
| 3564 | Town13 | InvadingTurn | Give_Way | night, rain 60%, fog 60% | 104 m |
| 3572 | Town13 | InvadingTurn | Give_Way | day, fog 3% | 102 m |
| 3575 | Town13 | InvadingTurn | Give_Way | day, rain 100%, fog 7% | 104 m |
| 3785 | Town13 | MergerIntoSlowTraffic | Merging | day, rain 100%, fog 7% | 222 m |
| 3800 | Town13 | MergerIntoSlowTraffic | Merging | night, rain 100%, fog 100% | 182 m |
| 24092 | Town13 | MergerIntoSlowTrafficV2 | Merging | day | 151 m |
| 3666 | Town13 | NonSignalizedJunctionRightTurn | Merging/Traffic_Signs | night, rain 100%, fog 3% | 71 m |
| 3670 | Town13 | NonSignalizedJunctionRightTurn | Merging/Traffic_Signs | night, rain 100%, fog 100% | 69 m |
| 3676 | Town13 | NonSignalizedJunctionRightTurn | Merging/Traffic_Signs | day, fog 3% | 69 m |
| 3697 | Town13 | OppositeVehicleTakingPriority | Emergency_Brake/Traffic_Signs | day, fog 2% | 73 m |
| 3457 | Town13 | ParkedObstacleTwoWays | Overtaking | night, rain 30%, fog 3% | 134 m |
| 3248 | Town13 | ParkingCrossingPedestrian | Emergency_Brake | night, fog 2% | 134 m |
| 3255* | Town13 | ParkingCrossingPedestrian | Emergency_Brake | day, fog 2% | 118 m |
| 3514* | Town13 | ParkingExit | Merging | night, rain 100%, fog 100% | 134 m |
| 3520 | Town13 | ParkingExit | Merging | day, fog 3% | 134 m |
| 3890 | Town13 | VanillaNonSignalizedTurn | Traffic_Signs | day, rain 60%, fog 50% | 59 m |
| 3904 | Town13 | VanillaNonSignalizedTurnEncounterStopsign | Traffic_Signs | night, rain 100%, fog 100% | 55 m |
| 3905 | Town13 | VanillaNonSignalizedTurnEncounterStopsign | Traffic_Signs | day, fog 2% | 60 m |
| 3865 | Town13 | VanillaSignalizedTurnEncounterRedLight | Traffic_Signs | day, fog 50% | 60 m |
| 3869 | Town13 | VanillaSignalizedTurnEncounterRedLight | Traffic_Signs | day, rain 100%, fog 7% | 57 m |
| 3876 | Town13 | VanillaSignalizedTurnEncounterRedLight | Traffic_Signs | night, rain 60%, fog 60% | 51 m |
| 3464 | Town13 | VehicleOpensDoorTwoWays | Overtaking | day, fog 2% | 134 m |
| 3472 | Town13 | VehicleOpensDoorTwoWays | Overtaking | day, rain 60%, fog 50% | 134 m |
| 3476 | Town13 | VehicleOpensDoorTwoWays | Overtaking | day, fog 50% | 134 m |
| 3482 | Town13 | VehicleOpensDoorTwoWays | Overtaking | night, fog 2% | 132 m |
| 3708 | Town13 | VehicleTurningRoute | Emergency_Brake/Traffic_Signs | day, rain 60%, fog 50% | 63 m |
| 3712 | Town13 | VehicleTurningRoute | Emergency_Brake/Traffic_Signs | day, rain 50%, fog 10% | 75 m |
| 3717 | Town13 | VehicleTurningRoute | Emergency_Brake/Traffic_Signs | night, rain 30%, fog 3% | 72 m |
| 3731 | Town13 | VehicleTurningRoutePedestrian | Emergency_Brake/Traffic_Signs | day, rain 100%, fog 7% | 75 m |
| 3737 | Town13 | VehicleTurningRoutePedestrian | Emergency_Brake/Traffic_Signs | day, rain 100%, fog 10% | 70 m |
| 3364 | Town13 | YieldToEmergencyVehicle | Give_Way | day, fog 3% | 104 m |
| 3373 | Town13 | YieldToEmergencyVehicle | Give_Way | day, rain 100%, fog 10% | 104 m |
| 3378 | Town13 | YieldToEmergencyVehicle | Give_Way | night, fog 2% | 104 m |
| 3380 | Town13 | YieldToEmergencyVehicle | Give_Way | night, rain 100%, fog 3% | 105 m |
| 25845 | Town15 | AccidentTwoWays | Overtaking | day, fog 2% | 106 m |
| 24333 | Town15 | DynamicObjectCrossing | Emergency_Brake | night, rain 100%, fog 100% | 134 m |
| 25439 | Town15 | HazardAtSideLane | Overtaking | day, fog 2% | 129 m |
| 25383 | Town15 | ParkedObstacle | Overtaking | day, rain 100%, fog 10% | 140 m |
| 28198* | Town15 | SignalizedJunctionLeftTurnEnterFlow | Merging/Traffic_Signs | day, rain 60%, fog 3% | 78 m |
| 27018 | Town15 | SignalizedJunctionRightTurn | Merging/Traffic_Signs | day, fog 2% | 74 m |
| 26405* | Town15 | StaticCutIn | Emergency_Brake | day, fog 2% | 136 m |

## Routes per town
Town01: 4, Town02: 4, Town03: 11, Town04: 12, Town05: 9, Town06: 6, Town07: 5, Town10HD: 4, Town11: 7, Town12: 104, Town13: 47, Town15: 7
