# Priority-Based Maneuver Coordination in Artery

This documentation accompanies Daniel Maksimovski's
[research repository](https://github.com/maksimovskid/artery-maneuver-coordination).
Start with the [installation guide](install.md) and [MCM user/developer guide](mcm_user_and_developer_guide.md).
The framework pages below retain upstream Artery background; this fork's MCM
scope and validation are described in the project README and implementation audit.
Report fork-specific problems in the
[project issue tracker](https://github.com/maksimovskid/artery-maneuver-coordination/issues).

## Upstream Artery framework

![Logo of Artery V2X Simulation Framework](assets/logo.png#only-light)
![Logo of Artery V2X Simulation Framework](assets/logo-dark.png#only-dark)

Artery enables V2X simulations based on ETSI ITS-G5 protocols like GeoNetworking and BTP.
Single vehicles can be equipped with multiple ITS-G5 services through Artery's middleware, which also provides common Facilities for these services.
Some basic services, such as Cooperative Awareness (CAMs) and Decentralized Notification (DENMs), are already included.


## What is your medicine?

You may wonder what it is all about those medical terms about blood vessels in a technical environment.
Artery started as an extension of [Veins](https://veins.car2x.org), whose name derives from "Vehicles in Network
Simulation".
Veins has put emphasis on WAVE, the U.S. counterpart of ITS-G5, at least back in 2014 when development on Artery has
started.
Though Artery is not a fancy acronym, we thought it is a nice play on words because "arteries" are just related to
"veins" as is ITS-G5 to WAVE.
Please note that you can run Artery without Veins when you employ the radio model from [INET](https://inet.omnetpp.org).


## License

Artery is made available under GPL version 2; see this repository's [COPYING](https://github.com/maksimovskid/artery-maneuver-coordination/blob/master/COPYING).
Bundled and submodule dependencies retain their own licenses and upstream notices.
Upstream framework contributions can also be proposed to [Artery](https://github.com/riebl/artery).
