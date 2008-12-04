/*
    Copyright 2008 Rutgers University
    Copyright 2008 John Asmuth

    This file is part of CRL.

    CRL is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    CRL is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with CRL.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * This file contains a set of functions and classes aimed at general C++
 * usefullness.
 */

#ifndef CPPUTIL_H
#define CPPUTIL_H

#include <stdexcept>
#include <execinfo.h>
#include <signal.h>
#include <exception>
#include <iostream>
#include <sstream>
#include <vector>
#include <sys/time.h>
#include <boost/shared_ptr.hpp>

namespace cpputil {

class Exception {
protected:
	void createTrace() {
		void* array[25];
		int nSize = backtrace(array, 25);
		char** symbols = backtrace_symbols(array, nSize);
		
		std::ostringstream os;
		for (int i = 1; i < nSize; i++) {
			os << i << " - " << symbols[i] << std::endl;
		}
		trace = os.str();

		free(symbols);
	}
	Exception(std::string name, std::string what) {
		this->name = name;
		if (what != "")
			this->what = what;
		createTrace();
    }
public:
	std::string name;
	std::string what;
	std::string trace;
	Exception(std::string what) {
		this->name = "Exception";
		this->what = what;
		createTrace();
    }
	operator std::string() {return what;}
};

class RangeException : public Exception {
public:
	RangeException(double v, double min, double max, std::string what="")
	: Exception("RangeException", "") {
		std::ostringstream os;
		if (what != "")
			os << what << " - ";
		os << v << " : [" << min << ", " << max << "]";
		this->what = os.str();
	}
	RangeException(int v, int min, int max, std::string what="")
	: Exception("RangeException", "") {
		std::ostringstream os;
		if (what != "")
			os << what << " - ";
		os << v << " : [" << min << ", " << max << "]";
		this->what = os.str();
	}
};

class SizeException : public Exception {
public:
	SizeException(int s1, int s2, std::string what="")
	: Exception("SizeException", "") {
		std::ostringstream os;
		if (what != "")
			os << what << " - ";
		os << "expected " << s1 << ", got " << s2;
		this->what = os.str();
	}
};

class IndexException : public Exception {
public:
	IndexException(int i, int r, std::string what="")
	: Exception("IndexException", "") {
		std::ostringstream os;
		if (what != "")
			os << what << " - ";
		os << i << " is out of range " << r;
		this->what = os.str();
	}
};

class IteratorException : public cpputil::Exception {
public:
	IteratorException(std::string what="")
	: Exception("IteratorException", what) {

	}
};

inline std::ostream& operator<<(std::ostream& os, const Exception& ex) {
	os << ex.name << " : " << ex.what << std::endl;
	os << ex.trace << std::endl;
	return os;
}

template <class T, class I>
class IndexedObject;

template <class T, class I>
class Indexer {
	IndexedObject<T,I>* _io;
	I _index;
public:
	Indexer(IndexedObject<T,I>* io, I index)
	: _io(io), _index(index) { }
	operator const T&() {return _io->getItem(_index);}
	T operator=(const T& t) {
		_io->setItem(_index, t);
		return t;
	}
};


template <class T, class I>
class IndexedObject {
public:
	Indexer<T,I> operator[](I index) {
		return Indexer<T,I>(this, index);
	}
	const T& operator[](I index) const {
		return getItem(index);
	}
	virtual void setItem(I index, T data) = 0;
	virtual const T& getItem(I index) const = 0;
	virtual T& getItem(I index) = 0;
};

template <class T, class I>
std::ostream& operator<<(std::ostream& os, Indexer<T,I> i) {
	return os << (T)i;
}

template <class T>
class Iterator {
public:
	virtual const T& next() = 0;
	virtual bool hasNext() = 0;
	virtual void reset() = 0;
	virtual ~Iterator() { }
};

template <class T>
class EmptyIterator : public Iterator<T> {
public:
	virtual const T& next() {
		throw IteratorException("calling next on empty iterator");
	}
	virtual bool hasNext() {
		return false;
	}
	virtual void reset() {
		
	}
	virtual ~EmptyIterator() { }
};

template <class T, class M>
class MapKeyIterator : public Iterator<T> {
private:
	M& _map;
	typename M::iterator _itr;
	typename M::iterator _end;
public:
	MapKeyIterator(M& map)
	: _map(map) {
		_itr = _map.begin();
		_end = _map.end();
	}
	MapKeyIterator(boost::shared_ptr<M> shared_map)
	: _map(*shared_map) {
		_itr = _map.begin();
		_end = _map.end();
	}
	virtual const T& next() {
		if (_itr == _end)
			throw IteratorException("Attempting to get next element of exhausted iterator");
		const T& t = _itr->first;
		_itr++;
		return t;
	}
	virtual bool hasNext() {
		return _itr != _end;
	}
	virtual void reset() {
		_itr = _map.begin();
	}
};

template <class T, class M>
class MapValueIterator : public Iterator<T> {
private:
	M& _map;
	typename M::iterator _itr;
	typename M::iterator _end;
public:
	MapValueIterator(M& map)
	: _map(map) {
		_itr = _map.begin();
		_end = _map.end();
	}
	MapValueIterator(boost::shared_ptr<M> shared_map)
	: _map(*shared_map) {
		_itr = _map.begin();
		_end = _map.end();
	}
	virtual const T& next() {
		if (_itr == _end)
			throw IteratorException("Attempting to get next element of exhausted iterator");
		const T& t = _itr->second;
		_itr++;
		return t;
	}
	virtual bool hasNext() {
		return _itr != _end;
	}
	virtual void reset() {
		_itr = _map.begin();
	}
};

template <class T, class C>
class ContainerIterator : public Iterator<T> {
private:
	C& _container;
	typename C::iterator _itr;
	typename C::iterator _end;
public:
	ContainerIterator(C& _container)
	: _container(_container) {
		_itr = _container.begin();
		_end = _container.end();
	}
	ContainerIterator(boost::shared_ptr<C> shared_container)
	: _container(*shared_container) {
		_itr = _container.begin();
		_end = _container.end();
	}
	virtual const T& next() {
		if (_itr == _end)
			throw IteratorException("Attempting to get next element of exhausted iterator");
		const T& t = *_itr;
		_itr++;
		return t;
	}
	virtual bool hasNext() {
		return _itr != _end;
	}
	virtual void reset() {
		_itr = _container.begin();
	}
};


template <class T, class C>
class SharedContainerIterator : public Iterator<T> {
private:
	boost::shared_ptr<C> _container;
	typename C::iterator _itr;
	typename C::iterator _end;
public:
	SharedContainerIterator(boost::shared_ptr<C> _container)
	: _container(_container) {
		_itr = _container->begin();
		_end = _container->end();
	}
	virtual const T& next() {
		if (_itr == _end)
			throw IteratorException("Attempting to get next element of exhausted iterator");
		const T& t = *_itr;
		_itr++;
		return t;
	}
	virtual bool hasNext() {
		return _itr != _end;
	}
	virtual void reset() {
		_itr = _container->begin();
	}
};

template <class T>
class VectorIterator : public ContainerIterator<T,std::vector<T> > {
private:
public:
	VectorIterator(std::vector<T>& _vec)
	: ContainerIterator<T,std::vector<T> >(_vec) { }
	VectorIterator(boost::shared_ptr<std::vector<T> > shared_vec)
	: ContainerIterator<T,std::vector<T> >(*shared_vec) { }
};



template <class T>
class Range {
private:
	T min;
	T max;
public:
	Range() { }
	Range(const Range& r)
	: min(r.min), max(r.max) { }
	Range(T min, T max)
	: min(min), max(max) { }
	bool check(T v) {
		return v>=min && v<=max;
	}
	void checkThrow(T v) {
		if (!check(v))
			throw RangeException(v, min, max);
	}
	const T& getMin() const {return min;}
	const T& getMax() const {return max;}
	T getSpan() const{return max-min;}
};

template <class T>
std::ostream& operator<<(std::ostream& os, std::vector<T> v) {
	os << "[";
	for (int i=0; i<v.size(); i++) {
		os << v[i];
		if (i != v.size()-1) {
			os << ", ";	
		}	
	}
	os << "]";
	return os;
}

inline double randDouble() {
	return ((double)rand()/((double)(RAND_MAX)+(double)(1)));
}

inline time_t time_in_milli() {
  struct timeval tv;
  gettimeofday(&tv, 0);
  return tv.tv_usec/1000+tv.tv_sec*1000;
}

}

#endif
