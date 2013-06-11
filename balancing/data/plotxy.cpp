#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <cmath>
#include <stdlib.h>
using namespace std;

int main (int argc, char ** argv)
{
	double wheelRadius=10.5*0.0254;
	double dt, v, theta, xdot, ydot, t=0.0, x=0.0, y=0.0;
  string line;
  ifstream infile;
	ofstream outfile;
	char filename[25];
	sprintf(filename, "balance-dump-%04d.dat", atoi(argv[1]));
  infile.open (filename);
	outfile.open ("xy-90.dat");
  while(!infile.eof()) // To get you all the lines.
  {
	  getline(infile, line);	
		if(line[0]=='#') {
			outfile << line << endl;
			continue;
		}
		vector <double> values;
		double ff;
		istringstream stm (line);
		while( stm >> ff  ) values.push_back(ff);
		if(values.size()<34) continue;	
		dt=values[0]-t;
		v=wheelRadius*values[19];
		theta=values[33];
		
		xdot=-v*sin(theta);
		ydot= v*cos(theta);
		
		t+=dt;
		x+=xdot*dt;
		y+=ydot*dt;
	
		outfile << line << ' ' << x << ' ' << y << endl;	
  }
  infile.close();
	outfile.close();
}
	
	// Find all x and y by integrating xdot and ydot

