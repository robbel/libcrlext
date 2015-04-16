#include <iostream>
#include <sstream>
#include <math.h>

#include "diastream.hpp"

using namespace std;


double mapx(double x) {
	if (x<-1)
		return 0;
	if (x>1)
		return 0;
	return 1-fabs(x);
}

string getColor(double x) {
	//cerr << "+getColor " << endl;
	double r = mapx(2*x);
	double b = mapx(2*(x-.5));
	double g = mapx(2*(x-1));
	
	int ri = (int)(r*255);
	int bi = (int)(b*255);
	int gi = (int)(g*255);
	
	//cerr << x << " " << r << " " << g << " " << b << endl;
	
	string cs = "#";
	const char* hex = "0123456789ABCDEF";
	cs += hex[ri/16];
	cs += hex[ri%16];
	cs += hex[gi/16];
	cs += hex[gi%16];
	cs += hex[bi/16];
	cs += hex[bi%16];
	
	//cerr << "-getColor" << endl;
	return cs;
}



DiaStatement DiaBeginGroup() {
	return DiaStatement("    <dia:group>");
}
DiaStatement DiaEndGroup() {
	return DiaStatement("    </dia:group>");
}
DiaStatement DiaEndLayer() {
	return DiaStatement("  </dia:layer>");
}
DiaStatement DiaEndDoc() {
	return DiaStatement("</dia:diagram>");
}

DiaLine::DiaLine(float sx, float sy, float dx, float dy, string color, float width, float gap)
: _sx(sx), _sy(sy), _dx(dx), _dy(dy), _color(color), _width(width), _gap(gap) {
	
}
string DiaLine::getStatement() const {
	ostringstream os;
	os << "      <dia:object type=\"Standard - Line\" version=\"0\" id=\"O3\">" << endl
	   << "        <dia:attribute name=\"obj_pos\">" << endl
	   << "          <dia:point val=\"" << _sx << "," << _sy << "\"/>" << endl
	   << "        </dia:attribute>" << endl
	   << "        <dia:attribute name=\"conn_endpoints\">" << endl
	   << "          <dia:point val=\"" << _sx << "," << _sy << "\"/>" << endl
	   << "          <dia:point val=\"" << _dx << "," << _dy << "\"/>" << endl
	   << "        </dia:attribute>" << endl
	   << "        <dia:attribute name=\"numcp\">" << endl
	   << "          <dia:int val=\"1\"/>" << endl
	   << "        </dia:attribute>" << endl
	   << "        <dia:attribute name=\"line_width\">" << endl
	   << "          <dia:real val=\"" << _width << "\"/>" << endl
	   << "        </dia:attribute>" << endl
	   << "        <dia:attribute name=\"absolute_start_gap\">" << endl
	   << "          <dia:real val=\"" << -1*_gap << "\"/>" << endl
	   << "        </dia:attribute>" << endl
	   << "        <dia:attribute name=\"absolute_end_gap\">" << endl
	   << "          <dia:real val=\"-" << _gap << "\"/>" << endl
	   << "        </dia:attribute>" << endl
	   << "        <dia:attribute name=\"line_color\">" << endl
	   << "          <dia:color val=\"" << _color << "\"/>" << endl
	   << "        </dia:attribute>" << endl
	   << "      </dia:object>" << endl;
	return os.str();
}

DiaArc::DiaArc(float sx, float sy, float dx, float dy,
                   float curve_distance, string color, float width)
: _sx(sx), _sy(sy), _dx(dx), _dy(dy), _curve_distance(curve_distance), _color(color), _width(width) {
 	
}

string DiaArc::getStatement() const {
	ostringstream os;
	os << "      <dia:object type=\"Standard - Arc\" version=\"0\" id=\"O42\">" << endl
	   << "        <dia:attribute name=\"obj_pos\">" << endl
	   << "          <dia:point val=\"" << _sx << "," << _sy << "\"/>" << endl
	   << "        </dia:attribute>" << endl
	   << "        <dia:attribute name=\"obj_bb\">" << endl
	   << "          <dia:rectangle val=\"33.935,12.0616;44.6431,15.963\"/>" << endl
	   << "        </dia:attribute>" << endl
	   << "        <dia:attribute name=\"conn_endpoints\">" << endl
	   << "          <dia:point val=\"" << _sx << "," << _sy << "\"/>" << endl
	   << "          <dia:point val=\"" << _dx << "," << _dy << "\"/>" << endl
	   << "        </dia:attribute>" << endl
	   << "        <dia:attribute name=\"arc_color\">" << endl
	   << "          <dia:color val=\"" << _color << "\"/>" << endl
	   << "        </dia:attribute>" << endl
	   << "        <dia:attribute name=\"curve_distance\">" << endl
	   << "          <dia:real val=\"" << _curve_distance << "\"/>" << endl
	   << "        </dia:attribute>" << endl
	   << "        <dia:attribute name=\"line_width\">" << endl
	   << "          <dia:real val=\"" << _width << "\"/>" << endl
	   << "        </dia:attribute>" << endl
	   << "        <dia:attribute name=\"end_arrow\">" << endl
	   << "          <dia:enum val=\"22\"/>" << endl
	   << "        </dia:attribute>" << endl
	   << "        <dia:attribute name=\"end_arrow_length\">" << endl
	   << "          <dia:real val=\"" << _width*3 << "\"/>" << endl
	   << "        </dia:attribute>" << endl
	   << "        <dia:attribute name=\"end_arrow_width\">" << endl
	   << "          <dia:real val=\"" << _width*3 << "\"/>" << endl
	   << "        </dia:attribute>" << endl
	   << "      </dia:object>" << endl;
	return os.str();
}

DiaArrow::DiaArrow(float sx, float sy, float dx, float dy)
: _sx(sx), _sy(sy), _dx(dx), _dy(dy) {
	
}
string DiaArrow::getStatement() const {
	ostringstream os;
	double ox = (_sx+_dx)*0.5;
	double oy = (_sy+_dy)*0.5;
	
	os << "    <dia:object type=\"Standard - Line\" version=\"0\" id=\"O10\">" << endl
	   << "      <dia:attribute name=\"obj_pos\">" << endl
	   << "        <dia:point val=\"" << ox << "," << oy << "\"/>" << endl
	   << "      </dia:attribute>" << endl
	   << "      <dia:attribute name=\"conn_endpoints\">" << endl
	   << "        <dia:point val=\"" << _sx << "," << _sy << "\"/>" << endl
	   << "        <dia:point val=\"" << _dx << "," << _dy << "\"/>" << endl
	   << "      </dia:attribute>" << endl
	   << "      <dia:attribute name=\"numcp\">" << endl
	   << "        <dia:int val=\"1\"/>" << endl
	   << "      </dia:attribute>" << endl
	   << "      <dia:attribute name=\"line_width\">" << endl
	   << "        <dia:real val=\"0.050000000000000003\"/>" << endl
	   << "      </dia:attribute>" << endl
	   << "      <dia:attribute name=\"end_arrow\">" << endl
	   << "        <dia:enum val=\"22\"/>" << endl
	   << "      </dia:attribute>" << endl
	   << "      <dia:attribute name=\"end_arrow_length\">" << endl
	   << "        <dia:real val=\"0.25\"/>" << endl
	   << "      </dia:attribute>" << endl
	   << "      <dia:attribute name=\"end_arrow_width\">" << endl
	   << "        <dia:real val=\"0.25\"/>" << endl
	   << "      </dia:attribute>" << endl
	   << "    </dia:object>" << endl;
	return os.str();
}

DiaBox::DiaBox(float sx, float sy, float dx, float dy, string fill_color, string border_color)
: _sx(sx), _sy(sy), _dx(dx), _dy(dy), _fill_color(fill_color), _border_color(border_color) {
	
}
std::string DiaBox::getStatement() const {
	ostringstream os;
	os << "    <dia:object type=\"Standard - Box\" version=\"0\" id=\"O2\">" << endl
	   << "      <dia:attribute name=\"obj_pos\">" << endl
	   << "        <dia:point val=\"" << _sx << "," << _sy << "\"/>" << endl
	   << "      </dia:attribute>" << endl
	   << "      <dia:attribute name=\"elem_corner\">" << endl
	   << "        <dia:point val=\"" << _sx << "," << _sy << "\"/>" << endl
	   << "      </dia:attribute>" << endl
	   << "      <dia:attribute name=\"elem_width\">" << endl
	   << "        <dia:real val=\"" << _dx-_sx << "\"/>" << endl
	   << "      </dia:attribute>" << endl
	   << "      <dia:attribute name=\"elem_height\">" << endl
	   << "        <dia:real val=\"" << _dy-_sy << "\"/>" << endl
	   << "      </dia:attribute>" << endl
	   << "      <dia:attribute name=\"border_color\">" << endl
	   << "        <dia:color val=\"" << _border_color.c_str() << "\"/>" << endl
	   << "      </dia:attribute>" << endl
	   << "      <dia:attribute name=\"inner_color\">" << endl
	   << "        <dia:color val=\"" << _fill_color.c_str() << "\"/>" << endl
	   << "      </dia:attribute>" << endl
	   << "      <dia:attribute name=\"show_background\">" << endl
	   << "        <dia:boolean val=\"true\"/>" << endl
	   << "        <dia:attribute name=\"absolute_start_gap\">" << endl
	   << "          <dia:real val=\"" << -.05 << "\"/>" << endl
	   << "        </dia:attribute>" << endl
	   << "        <dia:attribute name=\"absolute_end_gap\">" << endl
	   << "          <dia:real val=\"-" << .05 << "\"/>" << endl
	   << "        </dia:attribute>" << endl
	   << "      </dia:attribute>" << endl
	   << "      <dia:attribute name=\"corner_radius\">" << endl
	   << "        <dia:real val=\"1.1754943508222875e-38\"/>" << endl
	   << "      </dia:attribute>" << endl
	   << "    </dia:object>" << endl;
	return os.str();
}
	
DiaText::DiaText(float x, float y, string text, string color) 
: _x(x), _y(y), _text(text), _color(color) {
	
}
std::string DiaText::getStatement() const {
	ostringstream os;
	double dx = _x+0.25;
	double dy = _y+0.75;
	os << "      <dia:object type=\"Standard - Text\" version=\"1\" id=\"O8\">" << endl
	   << "        <dia:attribute name=\"obj_pos\">" << endl
	   << "          <dia:point val=\"" << dx << "," << dy << "\"/>" << endl
	   << "        </dia:attribute>" << endl
	   << "        <dia:attribute name=\"text\">" << endl
	   << "          <dia:composite type=\"text\">" << endl
	   << "            <dia:attribute name=\"string\">" << endl
	   << "              <dia:string>#" << _text.c_str() << "#</dia:string>" << endl
	   << "            </dia:attribute>" << endl
	   << "            <dia:attribute name=\"font\">" << endl
	   << "              <dia:font family=\"sans\" style=\"80\" name=\"Helvetica-Bold\"/>" << endl
	   << "            </dia:attribute>" << endl
	   << "            <dia:attribute name=\"height\">" << endl
	   << "              <dia:real val=\"1\"/>" << endl
	   << "            </dia:attribute>" << endl
	   << "            <dia:attribute name=\"color\">" << endl
	   << "              <dia:color val=\"" << _color.c_str() << "\"/>" << endl
	   << "            </dia:attribute>" << endl
	   << "            <dia:attribute name=\"alignment\">" << endl
	   << "              <dia:enum val=\"0\"/>" << endl
	   << "            </dia:attribute>" << endl
	   << "          </dia:composite>" << endl
	   << "        </dia:attribute>" << endl
	   << "        <dia:attribute name=\"valign\">" << endl
	   << "          <dia:enum val=\"3\"/>" << endl
	   << "        </dia:attribute>" << endl
	   << "      </dia:object>" << endl;
	return os.str();
}

DiaBeginDoc::DiaBeginDoc() {
	ostringstream os;
	os << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>" << endl
	   << "<dia:diagram xmlns:dia=\"http://www.lysator.liu.se/~alla/dia/\">" << endl
	   // diagram settings stuff
	   << "  <dia:diagramdata>" << endl
	   << "    <dia:attribute name=\"background\">" << endl
	   << "      <dia:color val=\"#ffffff\"/>" << endl
	   << "    </dia:attribute>" << endl
	   << "    <dia:attribute name=\"pagebreak\">" << endl
	   << "      <dia:color val=\"#000099\"/>" << endl
	   << "    </dia:attribute>" << endl
	   << "    <dia:attribute name=\"paper\">" << endl
	   << "      <dia:composite type=\"paper\">" << endl
	   << "        <dia:attribute name=\"name\">" << endl
	   << "          <dia:string>#Letter#</dia:string>" << endl
	   << "        </dia:attribute>" << endl
	   << "        <dia:attribute name=\"tmargin\">" << endl
	   << "          <dia:real val=\"2.5399999618530273\"/>" << endl
	   << "        </dia:attribute>" << endl
	   << "        <dia:attribute name=\"bmargin\">" << endl
	   << "          <dia:real val=\"2.5399999618530273\"/>" << endl
	   << "        </dia:attribute>" << endl
	   << "        <dia:attribute name=\"lmargin\">" << endl
	   << "          <dia:real val=\"2.5399999618530273\"/>" << endl
	   << "        </dia:attribute>" << endl
	   << "        <dia:attribute name=\"rmargin\">" << endl
	   << "          <dia:real val=\"2.5399999618530273\"/>" << endl
	   << "        </dia:attribute>" << endl
	   << "        <dia:attribute name=\"is_portrait\">" << endl
	   << "          <dia:boolean val=\"true\"/>" << endl
	   << "        </dia:attribute>" << endl
	   << "        <dia:attribute name=\"scaling\">" << endl
	   << "          <dia:real val=\"1\"/>" << endl
	   << "        </dia:attribute>" << endl
	   << "        <dia:attribute name=\"fitto\">" << endl
	   << "          <dia:boolean val=\"false\"/>" << endl
	   << "        </dia:attribute>" << endl
	   << "      </dia:composite>" << endl
	   << "    </dia:attribute>" << endl
	   << "    <dia:attribute name=\"grid\">" << endl
	   << "      <dia:composite type=\"grid\">" << endl
	   << "        <dia:attribute name=\"width_x\">" << endl
	   << "          <dia:real val=\"1\"/>" << endl
	   << "        </dia:attribute>" << endl
	   << "        <dia:attribute name=\"width_y\">" << endl
	   << "          <dia:real val=\"1\"/>" << endl
	   << "        </dia:attribute>" << endl
	   << "        <dia:attribute name=\"visible_x\">" << endl
	   << "          <dia:int val=\"1\"/>" << endl
	   << "        </dia:attribute>" << endl
	   << "        <dia:attribute name=\"visible_y\">" << endl
	   << "          <dia:int val=\"1\"/>" << endl
	   << "        </dia:attribute>" << endl
	   << "        <dia:composite type=\"color\"/>" << endl
	   << "      </dia:composite>" << endl
	   << "    </dia:attribute>" << endl
	   << "    <dia:attribute name=\"color\">" << endl
	   << "      <dia:color val=\"#d8e5e5\"/>" << endl
	   << "    </dia:attribute>" << endl
	   << "    <dia:attribute name=\"guides\">" << endl
	   << "      <dia:composite type=\"guides\">" << endl
	   << "        <dia:attribute name=\"hguides\"/>" << endl
	   << "        <dia:attribute name=\"vguides\"/>" << endl
	   << "      </dia:composite>" << endl
	   << "    </dia:attribute>" << endl
	   << "  </dia:diagramdata>" << endl;
	_what = os.str();
}
DiaBeginLayer::DiaBeginLayer(std::string name, bool visible) {
	ostringstream os;
	os << "  <dia:layer name=\"" << name.c_str() << "\" visible=\""
	   << (visible?"true":"false") << "\">" << endl;
	_what = os.str();
}
