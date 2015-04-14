/*
    Copyright 2015 Philipp Robbel

    License: http://www.gnu.org/licenses/lgpl LGPL version 3.0, or (at your option) any later version.

    The software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
 */

#ifndef LOGGER_HPP_
#define LOGGER_HPP_

#include <iostream>
#include <boost/assign/list_of.hpp>
#include <boost/unordered_map.hpp>

using boost::assign::map_list_of;

/*
 *  The minimum level to log (0 = Debug, ..., 4 = Fatal)
 */
#define MIN_LOG_LEVEL 0

//
// Extended version of simple console logger shown here:
//   http://stackoverflow.com/questions/8337300/c11-how-do-i-implement-convenient-logging-without-a-singleton
//

namespace logger {

enum class Level {
  Debug, // 0
  Info,  // 1
  Warn,  // 2
  Error, // 3
  Fatal  // 4
};

enum class Verbosity {
  LESS,
  MORE
};

const static boost::unordered_map<Level,const char*> LevelString = map_list_of
    (Level::Debug, "DEBUG")
    (Level::Info,  "INFO ")
    (Level::Warn,  "WARN ")
    (Level::Error, "ERROR")
    (Level::Fatal, "FATAL");

/**
 * Simple logger
 */
template<class T>
class Logger {
private:
  Level _level;
  T& _sink;
  Verbosity _verbosity;

public:
  Logger(Level l, T& ls, Verbosity v = Verbosity::LESS)
  : _level(l), _sink(ls), _verbosity(v) { }

  void operator()(std::string const& message,
                  char const* function,
                  char const* file,
                  int line)
  {
    _sink << "[" << LevelString.at(_level);
    if(_verbosity == Verbosity::MORE) {
      _sink << " " << file << ": " << function << "(), l." << line;
    }
    _sink << "]: " << message << "\n";
    _sink.flush();
  }
};

//
// Some console loggers
//

template<class T = std::ostream>
Logger<T>& Debug(T& out = std::cout) {
  static Logger<T> logger(Level::Debug, out, Verbosity::LESS);
  return logger;
}

template<class T = std::ostream>
Logger<T>& Info(T& out = std::cout) {
  static Logger<T> logger(Level::Info, out, Verbosity::LESS);
  return logger;
}

template<class T = std::ostream>
Logger<T>& Warn(T& out = std::cout) {
  static Logger<T> logger(Level::Warn, out, Verbosity::LESS);
  return logger;
}

template<class T = std::ostream>
Logger<T>& Error(T& out = std::cerr) {
  static Logger<T> logger(Level::Error, out, Verbosity::LESS);
  return logger;
}

template<class T = std::ostream>
Logger<T>& Fatal(T& out = std::cerr) {
  static Logger<T> logger(Level::Fatal, out, Verbosity::LESS);
  return logger;
}

} // namespace logger

//
// Accessibility macros
//

#define LOG(Logger_, Message_)                  \
  Logger_(                                      \
    static_cast<std::ostringstream&>(           \
      std::ostringstream().flush() << Message_  \
    ).str(),                                    \
    __FUNCTION__,                               \
    __FILE__,                                   \
    __LINE__                                    \
  );

#if defined NDEBUG || MIN_LOG_LEVEL > 0
#  define LOG_DEBUG(_) do {} while(0);
#else
#  define LOG_DEBUG(Message_) LOG(logger::Debug(), Message_)
#endif

#if MIN_LOG_LEVEL > 1
#  define LOG_INFO(_) do {} while(0);
#else
#  define LOG_INFO(Message_) LOG(logger::Info(), Message_)
#endif

#if MIN_LOG_LEVEL > 2
#  define LOG_WARN(_) do {} while(0);
#else
#  define LOG_WARN(Message_) LOG(logger::Warn(), Message_)
#endif

#if MIN_LOG_LEVEL > 3
#  define LOG_ERROR(_) do {} while(0);
#else
#  define LOG_ERROR(Message_) LOG(logger::Error(), Message_)
#endif

#if MIN_LOG_LEVEL > 4
#  define LOG_ERROR(_) do {} while(0);
#else
#  define LOG_FATAL(Message_) LOG(logger::Fatal(), Message_)
#endif

#endif
