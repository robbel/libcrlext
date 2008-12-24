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
#include <diastream.hpp>
#include <crl/vi.hpp>
#include <crl/ps.hpp>
#include <crl/uct.hpp>
#include <crl/fdomain.hpp>
#include "crl/mazes.hpp"

using namespace std;
using namespace crl;

void drawMaze(ostream& os, SlipMaze m, int offset_x, int offset_y, Domain domain, Planner planner) {
    os << DiaBeginGroup();
    
    os << DiaBeginGroup();
    for (size_t x=0; x<m->getWidth(); x++)
    	for (size_t y=0; y<m->getHeight(); y++) {
    		string s;
    		s += m->getTile(x, y);
    		if (s == "#") {
    			os << DiaBox(x+offset_x, y+offset_y, x+1+offset_x, y+1+offset_y, "#AAAAAA", "#AAAAAA");
    		}
    		else if (s == "S") {
    			os << DiaBox(x+offset_x, y+offset_y, x+1+offset_x, y+1+offset_y, "#00FF00", "#00FF00");
    		}
    		else if (s == "G") {
    			os << DiaBox(x+offset_x, y+offset_y, x+1+offset_x, y+1+offset_y, "#FF0000", "#FF0000");
    		}
    		else if (s == "X") {
    			os << DiaBox(x+offset_x, y+offset_y, x+1+offset_x, y+1+offset_y, "#000000", "#000000");
    		}
    		else if (s == "F") {
    			os << DiaBox(x+offset_x, y+offset_y, x+1+offset_x, y+1+offset_y, "#FF00FF", "#FF00FF");
    		}
    	}
    os << DiaEndGroup();
    
    os << DiaBeginGroup();
    os << DiaLine(offset_x, 0+offset_y, m->getWidth()+offset_x, 0+offset_y);
    os << DiaLine(offset_x, 0+offset_y, 0+offset_x, m->getHeight()+offset_y);
    os << DiaLine(offset_x, m->getHeight()+offset_y, m->getWidth()+offset_x, m->getHeight()+offset_y);
    os << DiaLine(m->getWidth()+offset_x, 0+offset_y, m->getWidth()+offset_x, m->getHeight()+offset_y);
    for (size_t x=0; x<m->getWidth(); x++)
    	for (size_t y=0; y<m->getHeight()-1; y++)
    		if (m->getWallSouth(x, y))
    			os << DiaLine(x+offset_x, y+1+offset_y, x+1+offset_x, y+1+offset_y);
    for (size_t x=0; x<m->getWidth()-1; x++)
    	for (size_t y=0; y<m->getHeight(); y++)
    		if (m->getWallEast(x, y))
    			os << DiaLine(x+1+offset_x, y+offset_y, x+1+offset_x, y+1+offset_y);
    os << DiaEndGroup();
    
    if (domain && planner) {
	    os << DiaBeginGroup();
	    for (size_t x=0; x<m->getWidth(); x++)
	    	for (size_t y=0; y<m->getHeight(); y++) {
	    		if (m->getTile(x, y) == 'G')
	    			continue;
	    		if (m->getTile(x, y) == '#')
	    			continue;
	    		State s = m->getState(x, y);
	    		
	    		Action a = planner->getAction(s);
	    		float dx = .5+x;
	    		float dy = .5+y;
	    		if (a.getFactor(0) == 0) {
	    			os << DiaArrow(dx+offset_x, dy+.25+offset_y, dx+offset_x, dy-.25+offset_y);
	    		}
	    		if (a.getFactor(0) == 1) {
	    			os << DiaArrow(dx-.25+offset_x, dy+offset_y, dx+.25+offset_x, dy+offset_y);
	    		}
	    		if (a.getFactor(0) == 2) {
	    			os << DiaArrow(dx+offset_x, dy-.25+offset_y, dx+offset_x, dy+.25+offset_y);
	    		}
	    		if (a.getFactor(0) == 3) {
	    			os << DiaArrow(dx+.25+offset_x, dy+offset_y, dx-.25+offset_x, dy+offset_y);
	    		}
	    	}
	    os << DiaEndGroup();
    }
    
    os << DiaEndGroup();
}

int main(int argc, char** argv) {
	if (argc != 4) {
		cout << "Usage: " << argv[0] << " <in maze> <in config> <out dia>" << endl;
		return 1;
	}
	ifstream in_maze(argv[1]);
	Maze m = readMaze(in_maze);
	ifstream in_cfg(argv[2]);
	SlipConfig cfg = readSlipConfig(in_cfg);
	SlipMaze sm(new _SlipMaze(m, cfg));
	
	FMDP mdp = getFMDP(sm->getMDP());
	
	Planner planner;
	
	
	VIPlanner vi_planner(new _FactoredVIPlanner(mdp->getDomain(), mdp, .001, 1));
	vi_planner->plan();
	planner = vi_planner;
	
	
	diastream os(argv[3]);
	
	os << DiaBeginDoc()
       << DiaBeginLayer("top");
    
	drawMaze(os, sm, 1, 1, sm->getDomain(), planner);
	
	os << DiaEndLayer() 
	   << DiaEndDoc();
	  
	   
	os.close();
}

