#include "daq.h"
#include <stdio.h>
#include <iostream>
#include <string.h>
#include <fstream>
using namespace std;
float64 sampleRate=300;
int32 timeout=2;
float64 vmin=-6;
float64 vmax=6;
float64 voltIn;
float64 output;
float64 y;
float64 gain = .75;
float64 u;
float64 e;
float64 input = 1;
float64 moveTo = 2;
float64 previous_e;
float64 previous_u;
float64 T = 0.1;
float64 w = 3.1415 * 2 * 200;
float64 u1 , u2 , u3 , utot, utotdisc1, utotdisc2, utotdisc3, previous_previous_u, previous_previous_e;
void main(){
	AI_Configuration(sampleRate, timeout, vmin, vmax);
	AO_Configuration(vmin, vmax);
	cout << "Please enter a position input (a float or int that is between -5 and 5): ";
	cin >> input;
		ofstream fout("Position_Data.txt");
	for(int i = 0; i<2300; i++){ 
		voltIn = Read_Voltage();
		e = input - voltIn;
		u = e * gain;
		previous_u = u;
		previous_previous_u=previous_u;
		previous_e = e;
		previous_previous_e=previous_u;
		previous_u = u;
		previous_e = e;
		utotdisc1 = 1.9215*previous_u - 0.9216*previous_previous_u + 80.5132*e - 160.1269*previous_e + 79.6165*previous_previous_e; 
		utotdisc2 = 1.9184*previous_u - 0.9185*previous_previous_u + 83.3333*e - 165.7333*previous_e + 82.4025*previous_previous_e; 
		utotdisc3 = 1.9968*previous_u - 0.9968*previous_previous_u + 84*e - 167.0648*previous_e + 83.0672*previous_previous_e;
		previous_u = u;
		previous_e = e;
	
		if (utotdisc1 >= 6){
			utotdisc1=6;
		}
		if (utotdisc2 >= 6){
			utotdisc2=6;
		}
		if (utotdisc3 >= 6){
			utotdisc3=6;
		}
		if (utotdisc1 <= -6){
			utotdisc1=-6;
		}
		if (utotdisc2 <= -6){
			utotdisc2=-6;
		}
		if (utotdisc3 <= -6){
			utotdisc3=-6;
		}
		Write_Voltage(u);
		printf("In: %f",input);
		printf("Pot: %f", voltIn);
		printf("\tU: %f\n " , utotdisc1);

		fout << utotdisc1 << " " << endl;


	}
	fout.close();
}