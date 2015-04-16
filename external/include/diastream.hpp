#ifndef DIA_GEN_H_
#define DIA_GEN_H_

#include <iostream>
#include <string>
#include <gzstream.h>

typedef ogzstream diastream;

class DiaStatement {
protected:
	std::string _what;
public:
	DiaStatement() { }
	DiaStatement(std::string what)
	: _what(what) { }
	virtual std::string getStatement() const {
		return _what;
	}
	virtual ~DiaStatement() { }
};

class DiaBeginLayer : public DiaStatement {
public:
	DiaBeginLayer(std::string name, bool visible=true);
};
class DiaBeginDoc : public DiaStatement {
public:
	DiaBeginDoc();
};

class DiaLine : public DiaStatement {
protected:
	float _sx, _sy, _dx, _dy;
	std::string _color;
	float _width, _gap;
public:
	DiaLine(float sx, float sy, float dx, float dy,
	        std::string color="#000000", float width=.1, float gap=.05);
	virtual std::string getStatement() const;
};
class DiaArrow : public DiaStatement {
protected:
	float _sx, _sy, _dx, _dy;
public:
	DiaArrow(float sx, float sy, float dx, float dy);
	virtual std::string getStatement() const;
};

class DiaArc : public DiaStatement {
protected:
	float _sx, _sy, _dx, _dy;
	float _curve_distance;
	std::string _color;
	float _width;
public:
	DiaArc(float sx, float sy, float dx, float dy,
	         float curve_distance, std::string color="#000000", float width=.1);
	virtual std::string getStatement() const;
};

class DiaBox : public DiaStatement {
protected:
	float _sx, _sy, _dx, _dy;
	std::string _fill_color;
	std::string _border_color;
public:
	DiaBox(float sx, float sy, float dx, float dy, std::string fill_color="#FFFFFF", std::string border_color="#000000");
	virtual std::string getStatement() const;
};

class DiaText : public DiaStatement {
protected:
	float _x, _y;
	std::string _text;
	std::string _color;
public:
	DiaText(float x, float y, std::string text, std::string color="#000000");
	virtual std::string getStatement() const;
};

DiaStatement DiaBeginGroup();
DiaStatement DiaEndGroup();
DiaStatement DiaEndLayer();
DiaStatement DiaEndDoc();

inline std::ostream& operator<<(std::ostream& os, const DiaStatement& ds) {
	os << ds.getStatement();
	return os;
}

#endif /*DIA_GEN_H_*/
