/*
    Copyright 2008 Rutgers University
    Copyright 2008 John Asmuth
    Copyright 2015 Philipp Robbel

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
 * usefulness.
 */

#ifndef CPPUTIL_H
#define CPPUTIL_H

#include <string.h>
#include <stdexcept>
#include <execinfo.h>
#include <signal.h>
#include <exception>
#include <iostream>
#include <sstream>
#include <vector>
#include <string>
#include <unordered_map>
#include <sys/time.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

namespace cpputil {

inline std::string getName(char*& rest) {
	int len = 0;
	while (isdigit(*rest)) {
		len *= 10;
		len += (*rest-'0');
		rest++;
	}
	std::string name = std::string(rest).substr(0, len);
	for (int i=0; i<len; i++)
		rest++;
	
	return name;
}

inline std::string getParam(char*& rest) {
	std::string s;
	while (*rest && !isdigit(*rest))
		rest++;
	if (!*rest)
		return std::string("");
	bool first = true;
	while (isdigit(*rest)) {
		if (!first)
			s += "::";
		s += getName(rest);
		first = false;
		
		if (*rest == 'I') {
			s += "<";
			bool first = true;
			while (*rest != 'E') {
				if (!first)
					s += ",";
				first = false;
				s += getParam(rest);
			}
			s += ">";
			rest++;
		}
	}
	if (s[0] == 'E')
		s = "";
	return s;
}

inline std::string translateSymbol(char* symbol) {
	std::string s;
	strtok(symbol, "(");
//	s += "[";
//	s += lib_file;
//	s += "]";
	char* rest = strtok(0, "+");
	if (!rest)
		return std::string("null symbol");
	if (*rest == '_') {
		s += getParam(rest);
		s += "(";
		std::string p;
		bool first = true;
		while ((p=getParam(rest)) != "") {
			if (!first)
				s += ",";
			first = false;
			s += p;
		}
		s += ")";
	}
	else if (*rest == ')') {
	    return std::string("unknown");
	}
	else {
		s += rest;
	}
	char* where = strtok(0, ")");
	if(where) {
		s += "+";
		s += where;
	}
	return s;
}

class Exception {
protected:
#if 0
	void createTrace() {
		void* array[25];
		int nSize = backtrace(array, 25);
		char** symbols = backtrace_symbols(array, nSize);
		
		std::ostringstream os;
		for (int i = 1; i < nSize; i++) {
			os << i << " - " << translateSymbol(symbols[i]) << std::endl;
		}
		trace = os.str();

		free(symbols);
	}
#endif
	Exception(std::string name, std::string what) {
		this->name = name;
		if (what != "")
			this->what = what;
#if 0
		createTrace();
#endif
    }
public:
	std::string name;
	std::string what;
	std::string trace;
	Exception(std::string what) {
		this->name = "Exception";
		this->what = what;
#if 0
		createTrace();
#endif
    }
        operator std::string() const {return what;}
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

class InvalidException : public cpputil::Exception {
public:
	InvalidException(std::string what="")
	: Exception("InvalidException", what) {

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

///
/// \brief Abstract forward iterator interface
///
template <class T>
class Iterator {
public:
	virtual const T& next() = 0;
	virtual bool hasNext() const = 0;
	virtual void reset() = 0;
};

template <class T>
class EmptyIterator : public Iterator<T> {
public:
	virtual const T& next() override {
		throw IteratorException("calling next on empty iterator");
	}
	virtual bool hasNext() const override {
		return false;
	}
	virtual void reset() override { }
};

///
/// \brief Forward iterator for keys in a std::map
///
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
	virtual const T& next() override {
		if (_itr == _end)
			throw IteratorException("Attempting to get next element of exhausted iterator");
		const T& t = _itr->first;
		++_itr;
		return t;
	}
	virtual bool hasNext() const override {
		return _itr != _end;
	}
	virtual void reset() override {
		_itr = _map.begin();
	}
};

///
/// \brief Forward iterator for values in a std::map
///
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
	virtual const T& next() override {
		if (_itr == _end)
			throw IteratorException("Attempting to get next element of exhausted iterator");
		const T& t = _itr->second;
		++_itr;
		return t;
	}
	virtual bool hasNext() const override {
		return _itr != _end;
	}
	virtual void reset() override {
		_itr = _map.begin();
	}
};

///
/// \brief Forward iterator for a value range from a map
///
template <class T, class M>
class MapValueRangeIterator : public Iterator<T> {
  typename M::iterator _begin;
  typename M::iterator _end;
  typename M::iterator _itr;
public:
  MapValueRangeIterator(typename M::iterator begin, typename M::iterator end)
  : _begin(begin), _end(end), _itr(begin) { }
  MapValueRangeIterator(std::pair<typename M::iterator, typename M::iterator> iters)
  : _begin(iters.first), _end(iters.second), _itr(iters.first) { }

  virtual const T& next() override {
    if (_itr == _end)
      throw IteratorException("Attempting to get next element of exhausted iterator");
    const T& t = _itr->second;
    ++_itr;
    return t;
  }
  virtual bool hasNext() const override {
    return _itr != _end;
  }
  virtual void reset() override {
    _itr = _begin;
  }
};

///
/// \brief Forward iterator for container type C storing elements of type T.
///
template <class T, class C>
class ContainerIterator : public Iterator<T> {
protected:
	const C& _container;
	typename C::const_iterator _itr;
	typename C::const_iterator _end;
	typedef typename C::size_type size_type;
public:
	ContainerIterator(const C& _container)
	: _container(_container) {
	  _itr = _container.begin();
	  _end = _container.end();
	}
	ContainerIterator(boost::shared_ptr<C> shared_container)
	: _container(*shared_container) {
	  _itr = _container.cbegin();
	  _end = _container.cend();
	}
	virtual const T& next() override {
		if (_itr == _end)
			throw IteratorException("Attempting to get next element of exhausted iterator");
		const T& t = *_itr;
		++_itr;
		return t;
	}
	virtual bool hasNext() const override {
		return _itr != _end;
	}
	virtual void reset() override {
		_itr = _container.begin();
	}
};

///
/// \brief Forward iterator for templated container that's wrapped in a shared_ptr.
/// \note Unlike the \a ContainerIterator the reference count of the shared_ptr is incremented.
///
template <class T, class C>
class SharedContainerIterator : public Iterator<T> {
protected:
	boost::shared_ptr<C> _container;
	typename C::const_iterator _itr;
	typename C::const_iterator _end;
	typedef typename C::size_type size_type;
public:
	SharedContainerIterator(boost::shared_ptr<C> _container)
	: _container(_container) {
		_itr = _container->cbegin();
		_end = _container->cend();
	}
	virtual const T& next() override {
		if (_itr == _end)
			throw IteratorException("Attempting to get next element of exhausted iterator");
		const T& t = *_itr;
		++_itr;
		return t;
	}
	virtual bool hasNext() const override {
		return _itr != _end;
	}
	virtual void reset() override {
		_itr = _container->begin();
	}
};

///
/// \brief Forward iterator for std::vector storing elements of type T.
///
template <class T>
class VectorIterator : public ContainerIterator<T,std::vector<T> > {
private:
public:
	VectorIterator(const std::vector<T>& _vec)
	: ContainerIterator<T,std::vector<T> >(_vec) { }
	VectorIterator(boost::shared_ptr<std::vector<T> > shared_vec)
	: ContainerIterator<T,std::vector<T> >(*shared_vec) { }
	// direct iterator access (for optimizations)
	typename std::vector<T>::const_iterator& get() {
	    return this->_itr;
	}
	const typename std::vector<T>::const_iterator& end() {
	    return this->_end;
	}
};

///
/// \brief Class for denoting a range [min,max]
///
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
	bool check(T v) const {
		return v>=min && v<=max;
	}
	void checkThrow(T v) const {
		if (!check(v))
			throw RangeException(v, min, max);
	}
	const T& getMin() const {return min;}
	const T& getMax() const {return max;}
	///
	/// \brief The span of the range ]min,max]
	/// \note Off-by-one compared to [min,max], e.g. during array size computations.
	/// \todo XXX figure out if acrobat_agent uses this correctly
	///
	T getSpan() const {return max-min;}
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

///
/// \brief return a random number in [0,1)
/// \note initialize seed once before usage
///
inline double randDouble() {
	return ((double)rand()/((double)(RAND_MAX)+(double)(1)));
}

inline time_t time_in_milli() {
  struct timeval tv;
  gettimeofday(&tv, 0);
  return tv.tv_usec/1000+tv.tv_sec*1000;
}

///
/// \brief An approximate comparison of two floating point numbers
///
template<class T>
bool approxEq(const T& a, const T& b, T eps = std::numeric_limits<T>::epsilon()) {
    return std::fabs(a - b) < eps;
}

///
/// \brief True iff val in [min,max)
///
template<class T>
bool in_interval(const T& val, const T& min, const T& max) {
    return val >= min && val < max;
}
template<class T>
bool in_pos_interval(const T& val, const T& max) {
    return in_interval<T>(val, 0, max);
}

///
/// \brief Check whether two collections intersect
///
template <class InputIterator1, class InputIterator2>
bool has_intersection (InputIterator1 first1, InputIterator1 last1,
                       InputIterator2 first2, InputIterator2 last2) {
  while (first1!=last1 && first2!=last2) {
    if (*first1<*first2) ++first1;
    else if (*first2<*first1) ++first2;
    else {
        return true;
    }
  }
  return false;
}

///
/// \brief Create a vector of consecutive values [start,start+size)
///
template<class T>
std::vector<T> ordered_vec(typename std::vector<T>::size_type size, T start = 0) {
  std::vector<T> order(size);
  std::iota(order.begin(), order.end(), start);
  return order;
}

///
/// \brief Inverse mapping
/// For example, {1,5,9} -> [1->0, 5->1, 9->2]
///
template<class T>
class inverse_map {
private:
  const typename std::vector<T>::size_type dom_size;
  std::unordered_map<T,typename std::vector<T>::size_type> _uo;
  /// \brief True if this is effectively an identity mapping (optimization)
  bool _identity;
public:
  explicit inverse_map(const std::vector<T>& dom)
  : dom_size(dom.size()), _uo(dom_size), _identity(true) {
    for(typename std::vector<T>::size_type i = 0; i < dom.size(); i++) {
        if(i != dom[i]) {
          _identity = false;
        }
        _uo.insert({dom[i],i});
    }
  }
  void append(T i) {
    _identity = false;
    _uo.insert({i, _uo.size()});
  }
  typename std::vector<T>::size_type operator()(T i) const {
    if(!_identity) {
      return _uo.at(i);
    }
    else /*if(i < dom_size)*/ {
      return i;
    }
//    else {
//      throw IndexException(i, dom_size, "inverse_map");
//    }
  }
  const std::unordered_map<T,typename std::vector<T>::size_type>& map() const {
      return _uo;
  }
};

}

#endif
