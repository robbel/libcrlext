/*
    Copyright 2008 Rutgers University
    Copyright 2008 John Asmuth

    This file is part of mazes.

    mazes is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    mazes is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with strxml.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <iostream>
#include <fstream>
#include "crl/mazes.hpp"

using namespace std;
using namespace crl;

int main(int argc, char** argv) {
	ifstream is(argv[1]);
	Maze m = readMaze(is);
	
	cout << "(define (problem Test)" << endl
		 << "\t(:domain maze)" << endl
		 << "\t(:objects" << endl
		 << "\t\tt - taxi" << endl
		 << "\t\tw - wall" << endl
		 << "\t\t";
	int i=0;
	for (size_t x=0; x<m->getWidth(); x++)
		for (size_t y=0; y<m->getHeight(); y++) {
			cout << "c" << x << "-" << y << " "; 
			i++;
			if (i%10==0)
				cout << endl << "\t\t";
		}
	cout << "- cell" << endl
		 << "\t)" << endl
		 << "\t(:init" << endl;
	for (size_t x=0; x<m->getWidth(); x++)
		for (size_t y=0; y<m->getHeight(); y++) {
			cout << "\t\t;;cell " << x << "-" << y << endl;
			if (y>0)
				cout << "\t\t(southOf c" << x << "-" << y << " c" << x << "-" << (y-1) << ")" << endl;
			if (y<m->getHeight()-1)
				cout << "\t\t(northOf c" << x << "-" << y << " c" << x << "-" << (y+1) << ")" << endl;
			if (x>0)
				cout << "\t\t(eastOf c" << x << "-" << y << " c" << (x-1) << "-" << y << ")" << endl;
			if (x<m->getWidth()-1)
				cout << "\t\t(westOf c" << x << "-" << y << " c" << (x+1) << "-" << y << ")" << endl;
			if (m->getWallNorth(x, y))
				cout << "\t\t(touch_N c" << x << "-" << y << " w)" << endl;
			if (m->getWallEast(x, y))
				cout << "\t\t(touch_E c" << x << "-" << y << " w)" << endl;
			if (m->getWallSouth(x, y))
				cout << "\t\t(touch_S c" << x << "-" << y << " w)" << endl;
			if (m->getWallWest(x, y))
				cout << "\t\t(touch_W c" << x << "-" << y << " w)" << endl;
		}
	for (size_t x=0; x<m->getWidth(); x++)
		for (size_t y=0; y<m->getHeight(); y++) {
			if (m->getTile(x, y) == 'S')
				cout << "\t\t(at t c" << x << "-" << y << ")" << endl;	
		}
	cout << "\t)" << endl
		 << "\t(:goal" << endl;
	for (size_t x=0; x<m->getWidth(); x++)
		for (size_t y=0; y<m->getHeight(); y++) {
			if (m->getTile(x, y) == 'G')
				cout << "\t\t(at t c" << x << "-" << y << ")" << endl;	
		}
	cout << "\t)" << endl
		 << ")" << endl;
}
