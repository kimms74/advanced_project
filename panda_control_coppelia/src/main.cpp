#include <iostream>
#include <string>
#include <conio.h>
#include "coppelia_bridge.h"

#include "controller.h"

using namespace std;



int main()
{
	CoppeliaBridge cb(CoppeliaBridge::CTRL_POSITION); // Position controlled 
	const double hz = 100;
	//CoppeliaBridge cb(CoppeliaBridge::CTRL_TORQUE); // Torque controlled
	//const double hz = 1000;
	ArmController ac(hz);
	bool is_simulation_run = true;
	bool exit_flag = false;
	bool is_first = true;

	while (cb.simConnectionCheck() && !exit_flag)
	{
		cb.read();	//coppeliasim bridge

		//position mode
		ac.readData(cb.getPosition(), cb.getVelocity());
		if (is_first)
		{
			cb.simLoop();
			cb.read();
			ac.readData(cb.getPosition(), cb.getVelocity());
			cout << "Initial q: " << cb.getPosition().transpose() << endl;
			is_first = false;
			ac.initPosition();
		}

		////torque mode
		//ac.readData(cb.getPosition(), cb.getVelocity(), cb.getTorque());
		//if (is_first)
		//{
		//	cb.simLoop();
		//	cb.read();
		//	ac.readData(cb.getPosition(), cb.getVelocity(), cb.getTorque());
		//	cout << "Initial q: " << cb.getPosition().transpose() << endl;
		//	is_first = false;
		//	ac.initPosition();
		//}

		if (_kbhit())	//key board hit
		{
			int key = _getch();

			switch (key)
			{
			// Implement with user input
			case 'i':
				ac.setMode("joint_ctrl_init");
				break;
			case 'h':
				ac.setMode("joint_ctrl_home");
				break;
			case 't':
				ac.setMode("torque_ctrl_dynamic");
				break;

			case 'a':
				ac.setMode("torque_ctrl_init1");
				break;
			case 's':
				ac.setMode("torque_ctrl_init2");
				break;


			case '0':
				ac.setMode("project_test");
				break;
			case '1':
				ac.setMode("project_home");
				break;
			case '2':
				ac.setMode("project_init");
				break;
			case '3':
				ac.setMode("project_function");
				break;
			case '4':
				ac.setMode("astar");
				break;


			case '\t':
				if (is_simulation_run) {
					cout << "Simulation Pause" << endl;
					is_simulation_run = false;
				}
				else {
					cout << "Simulation Run" << endl;
					is_simulation_run = true;
				}
				break;

			case 'q':
				is_simulation_run = false;
				exit_flag = true;
				break;
			default:
				break;
			}
		}

		if (is_simulation_run) {
			ac.compute();
			cb.setDesiredPosition(ac.getDesiredPosition());
			cb.setDesiredTorque(ac.getDesiredTorque());
		
			cb.write();
			cb.simLoop();
		}
	}
		
	return 0;
}
