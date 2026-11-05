/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Testable.h
 * @brief   Concept check for values that can be used in unit tests
 * @author  Frank Dellaert
 *
 * The necessary functions to implement for Testable are defined
 * below with additional details as to the interface.
 * The concept checking function will check whether or not
 * the function exists in derived class and throw compile-time errors.
 *
 * print with optional string naming the object
 *     void print(const std::string& name) const = 0;
 *
 * equality up to tolerance
 * tricky to implement, see PriorFactor for an example
 * equals is not supposed to print out *anything*, just return true|false
 *     bool equals(const Derived& expected, double tol) const = 0;
 *
 */

// \callgraph

#pragma once
#ifdef USING_BOOST_DEV
#include <boost/concept_check.hpp>
#else
//  #define BOOST_CONCEPT_ASSERT(x)  static_assert((x)::value,"");
//  #define BOOST_RANGE_CONCEPT_ASSERT(x)  static_assert((x), "BOOST_CONCEPT_ASSERT !")
#endif
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <type_traits>

#define GTSAM_PRINT(x)((x).print(#x))

namespace gtsam {

  // Forward declaration
  template <typename T> struct traits;

  /**
   * A testable concept check that should be placed in applicable unit
   * tests and in generic algorithms.
   *
   * See macros for details on using this structure
   * @ingroup base
   * @tparam T is the objectype this constrains to be testable - assumes print() and equals()
   */
  #ifdef USING_BOOST_DEV
  template <class T>
  class IsTestable {
    T t;
    bool r1,r2;
  public:

    BOOST_CONCEPT_USAGE(IsTestable) {
      // check print function, with optional string
      traits<T>::Print(t, std::string());
      traits<T>::Print(t);

      // check print, with optional threshold
      double tol = 1.0;
      r1 = traits<T>::Equals(t,t,tol);
      r2 = traits<T>::Equals(t,t);
    }
  }; // \ Testable
  #else
// 假设traits是一个已经定义好的模板结构体
template <class T>
struct traits {
  static void Print(const T& t, const std::string& str);
  static void Print(const T& t);
  static bool Equals(const T& t1, const T& t2, double tol);
  static bool Equals(const T& t1, const T& t2);
};

// 辅助类型，用于检查成员函数是否存在
template <typename T>
class HasPrintFunction {
  typedef char one;
  struct two { char x[2]; };

  template <typename C> static one test( decltype(&C::Print) ) ;
  template <typename C> static two test(...);

public:
  enum { value = sizeof(test<traits<T>>(0)) == sizeof(char) };
};

template <typename T>
class HasEqualsFunction {
  typedef char one;
  struct two { char x[2]; };

  template <typename C> static one test( decltype(&C::Equals) ) ;
  template <typename C> static two test(...);

public:
  enum { value = sizeof(test<traits<T>>(0)) == sizeof(char) };
};

// IsTestable类模板
template <class T>
class IsTestable {
  static_assert(HasPrintFunction<T>::value, "Type T must have a Print function in traits.");
  static_assert(HasEqualsFunction<T>::value, "Type T must have an Equals function in traits.");

  T t;
  bool r1, r2;

public:
    static constexpr bool value = HasPrintFunction<T>::value && HasEqualsFunction<T>::value;

    // 这里我们使用了构造函数来替代BOOST_CONCEPT_USAGE
  IsTestable() {
    // check print function, with optional string
    traits<T>::Print(t, std::string());
    traits<T>::Print(t);

    // check print, with optional threshold
    double tol = 1.0;
    r1 = traits<T>::Equals(t, t, tol);
    r2 = traits<T>::Equals(t, t);
  }
};
  #endif

  inline void print(float v, const std::string& s = "") {
    std::cout << (s.empty() ? s : s + " ") << v << std::endl;
  }
  inline void print(double v, const std::string& s = "") {
    std::cout << (s.empty() ? s : s + " ") << v << std::endl;
  }

  /** Call equal on the object */
  template<class T>
  inline bool equal(const T& obj1, const T& obj2, double tol) {
    return traits<T>::Equals(obj1,obj2, tol);
  }

  /** Call equal without tolerance (use default tolerance) */
  template<class T>
  inline bool equal(const T& obj1, const T& obj2) {
    return traits<T>::Equals(obj1,obj2);
  }

  /**
   * This template works for any type with equals
   */
  template<class V>
  bool assert_equal(const V& expected, const V& actual, double tol = 1e-9) {
    if (traits<V>::Equals(actual,expected, tol))
      return true;
    printf("Not equal:\n");
    traits<V>::Print(expected,"expected:\n");
    traits<V>::Print(actual,"actual:\n");
    return false;
  }

  /**
   * Template to create a binary predicate
   */
  template<class V>
  struct equals : public std::function<bool(const V&, const V&)> {
    double tol_;
    equals(double tol = 1e-9) : tol_(tol) {}
    bool operator()(const V& expected, const V& actual) {
      return (traits<V>::Equals(actual, expected, tol_));
    }
  };

  /**
   * Binary predicate on shared pointers
   */
  template<class V>
  struct equals_star : public std::function<bool(const std::shared_ptr<V>&, const std::shared_ptr<V>&)> {
    double tol_;
    equals_star(double tol = 1e-9) : tol_(tol) {}
    bool operator()(const std::shared_ptr<V>& expected, const std::shared_ptr<V>& actual) {
      if (!actual && !expected) return true;
      return actual && expected && traits<V>::Equals(*actual,*expected, tol_);
    }
  };
#ifdef USING_BOOST_DEV
  /// Requirements on type to pass it to Testable template below
  template<typename T>
  struct HasTestablePrereqs {

    BOOST_CONCEPT_USAGE(HasTestablePrereqs) {
      t->print(str);
      b = t->equals(*s,tol);
    }

    T *t, *s; // Pointer is to allow abstract classes
    bool b;
    double tol;
    std::string str;
  };

  /// A helper that implements the traits interface for GTSAM types.
  /// To use this for your gtsam type, define:
  /// template<> struct traits<Type> : public Testable<Type> { };
  template<typename T>
  struct Testable {

    // Check that T has the necessary methods
    BOOST_CONCEPT_ASSERT((HasTestablePrereqs<T>));

    static void Print(const T& m, const std::string& str = "") {
      m.print(str);
    }
    static bool Equals(const T& m1, const T& m2, double tol = 1e-8) {
      return m1.equals(m2, tol);
    }
  };
  #else
  template<typename T>
  struct HasTestablePrereqs {
      static_assert(HasPrintFunction<T>::value, "Type T must implement a print method with the signature: void print(const std::string&) const");
      static_assert(HasPrintFunction<T>::value, "Type T must implement an equals method with the signature: bool equals(const T&, double) const");

      // Constructor to perform the checks at compile time
      HasTestablePrereqs() {
          T* t = nullptr; // Using pointers to allow for abstract classes
          T* s = nullptr;
          double tol;
          std::string str;

          // The following lines are not actually executed, but they are used to ensure
          // at compile time that the methods exist.
          t->print(str);
          bool b = t->equals(*s, tol);
      }
  };
template<typename T>
struct Testable {
    // The constructor will trigger the static_assert checks in HasTestablePrereqs
    Testable() {
        static_assert(HasTestablePrereqs<T>::value, "Type T must satisfy HasTestablePrereqs");
    }

    static void Print(const T& m, const std::string& str = "") {
        m.print(str);
    }
    static bool Equals(const T& m1, const T& m2, double tol = 1e-8) {
        return m1.equals(m2, tol);
    }
};
  #endif

} // \namespace gtsam

/**
 * Macros for using the TestableConcept
 *  - An instantiation for use inside unit tests
 *  - A typedef for use inside generic algorithms
 *
 * NOTE: intentionally not in the gtsam namespace to allow for classes not in
 * the gtsam namespace to be more easily enforced as testable
 * @deprecated please use BOOST_CONCEPT_ASSERT and
 */
#define GTSAM_CONCEPT_TESTABLE_INST(T) template class gtsam::IsTestable<T>;
#define GTSAM_CONCEPT_TESTABLE_TYPE(T) using _gtsam_Testable_##T = gtsam::IsTestable<T>;
