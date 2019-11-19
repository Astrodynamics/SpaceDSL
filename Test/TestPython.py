from PySpaceDSL import *
import numpy as np

print("SpaceDSL Test Run!")


def main():
	# Initial Data
	initial_time = UTCCalTime(2018, 1, 4, 16, 58, 11.1)

	vehicle1_cart0 = CartState(-5.04649e+06, -3.53951e+06, -2.44795e+06, 4954.67, -4008.83, -4417.73)
	vehicle1_mass = 1000.0

	vehicle2_cart0 = CartState(-5.65484e+06, -3.96619e+06, -2.74305e+06, 4680.57, -3787.06, -4173.34)
	vehicle2_mass = 1500.0
	# Mission Start
	test_mission = Mission()
	test_mission.InsertSpaceVehicle('The First Vehicle', initial_time, vehicle1_cart0, vehicle1_mass, 2.2, 10, 1.0, 10)
	test_mission.InsertSpaceVehicle('The Second Vehicle', initial_time, vehicle2_cart0, vehicle2_mass, 2.2, 20, 1.0, 20)
	test_mission.InsertFacility('Facility1', -75.5966 * DegToRad(), 40.0386 * DegToRad(), 0, 10 * DegToRad())
	thirdGravSign = ThirdBodyGravitySign()
	thirdGravSign.bIsUseSunGrav = True
	thirdGravSign.bIsUseMoonGrav = True
	kp = 3.0
	ap = np.zeros(7)
	ap[0] = GeomagneticKpToAp(kp)

	test_mission.SetEnvironment(SolarSysStarType.E_Earth, GravityModel.GravModelType.E_EGM08Model, 20, 20,
	                            thirdGravSign, GeodeticCoordSystem.GeodeticCoordType.E_WGS84System,
	                            AtmosphereModel.AtmosphereModelType.E_NRLMSISE00Atmosphere,
	                            150, 150, ap, True, True)
	test_mission.SetPropagator(IntegMethodType.E_RungeKutta4, 60.0)
	test_mission.SetMissionSequence(initial_time, 86400)
	test_mission.Start(True)

	print('------First Calculation Finished ------')
	test_mission.CalMissionAccessData()
	accessListMap = test_mission.GetAccessData()
	print(accessListMap)

	# CZML File Wirte;
	script = CZMLScript()
	script.Initializer("PyTestData.czml", test_mission)
	script.WirteCZML()
	print("CZML Output Finished!")


if __name__ == "__main__":
	main()
