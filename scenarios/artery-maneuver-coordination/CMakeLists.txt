add_opp_run(artery_maneuver_coordination CONFIG omnetpp.ini)

if(WITH_ENVMOD)
    add_opp_test(artery_maneuver_coordination SUFFIX envmod CONFIG envmod SIMTIME_LIMIT 40s)
endif()
