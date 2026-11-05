/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Group.h
 *
 * @brief Concept check class for variable types with Group properties
 * @date November, 2011
 * @author Alex Cunningham
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/base/Testable.h>
#include <utility>

#ifdef USING_BOOST_DEV
#include <boost/concept_check.hpp>
#include <boost/concept/requires.hpp>
#include <boost/type_traits/is_base_of.hpp>
#include <boost/static_assert.hpp>
#else
  #include <type_traits>
#endif

namespace gtsam {

/// tag to assert a type is a group
struct group_tag {};

/// Group operator syntax flavors
struct multiplicative_group_tag {};
struct additive_group_tag {};

template <typename T> struct traits;

/**
 * Group Concept
 */
template<typename G>
class IsGroup {
public:
  typedef typename traits<G>::structure_category structure_category_tag;
  typedef typename traits<G>::group_flavor flavor_tag;
  //typedef typename traits<G>::identity::value_type identity_value_type;
#ifdef USING_BOOST_DEV
  BOOST_CONCEPT_USAGE(IsGroup) {
    static_assert(
        (std::is_base_of<group_tag, structure_category_tag>::value),
        "This type's structure_category trait does not assert it as a group (or derived)");
    e = traits<G>::Identity();
    e = traits<G>::Compose(g, h);
    e = traits<G>::Between(g, h);
    e = traits<G>::Inverse(g);
    operator_usage(flavor);
    // todo: how do we test the act concept? or do we even need to?
  }
#else
  static constexpr bool value = (std::is_base_of<group_tag, structure_category_tag>::value);

  // Constructor to perform the checks at compile time
  IsGroup() {
    // Compile-time check to ensure that structure_category_tag is derived from group_tag
//    static_assert(
//        std::is_base_of<group_tag, structure_category_tag>::value,
//        "This type's structure_category trait does not assert it as a group (or derived)"
//    );

    // The following lines are not actually executed, but they are used to ensure
    // at compile time that the methods exist and can be called with the expected parameters.
    e = traits<G>::Identity();
    e = traits<G>::Compose(g, h);
    e = traits<G>::Between(g, h);
    e = traits<G>::Inverse(g);
  }
#endif
private:
  void operator_usage(multiplicative_group_tag) {
    e = g * h;
    //e = -g; // todo this should work, but it is failing for Quaternions
  }
  void operator_usage(additive_group_tag) {
    e = g + h;
    e = h - g;
    e = -g;
  }

  flavor_tag flavor;
  G e, g, h;
  bool b;
};

/// Check invariants
#ifdef USING_BOOST_DEV
template<typename G>
BOOST_CONCEPT_REQUIRES(((IsGroup<G>)),(bool)) //
check_group_invariants(const G& a, const G& b, double tol = 1e-9) {
  G e = traits<G>::Identity();
  return traits<G>::Equals(traits<G>::Compose(a, traits<G>::Inverse(a)), e, tol)
      && traits<G>::Equals(traits<G>::Between(a, b), traits<G>::Compose(traits<G>::Inverse(a), b), tol)
      && traits<G>::Equals(traits<G>::Compose(a, traits<G>::Between(a, b)), b, tol);
}
#else

template<typename G, typename = void>
struct is_group : std::false_type {};

template<typename G>
struct is_group<G, decltype(IsGroup<G>(), void())> : std::true_type {};

// Now we can define the check_group_invariants function template using std::enable_if to
// ensure that G is a group.

template<typename G>
typename std::enable_if<is_group<G>::value, bool>::type
check_group_invariants(const G& a, const G& b, double tol = 1e-9) {
  G e = traits<G>::Identity();
  return traits<G>::Equals(traits<G>::Compose(a, traits<G>::Inverse(a)), e, tol)
      && traits<G>::Equals(traits<G>::Between(a, b), traits<G>::Compose(traits<G>::Inverse(a), b), tol)
      && traits<G>::Equals(traits<G>::Compose(a, traits<G>::Between(a, b)), b, tol);
}

#endif
namespace internal {

/// A helper class that implements the traits interface for multiplicative groups.
/// Assumes existence of identity, operator* and inverse method
template<class Class>
struct MultiplicativeGroupTraits {
  typedef group_tag structure_category;
  typedef multiplicative_group_tag group_flavor;
  static Class Identity() { return Class::Identity(); }
  static Class Compose(const Class &g, const Class & h) { return g * h;}
  static Class Between(const Class &g, const Class & h) { return g.inverse() * h;}
  static Class Inverse(const Class &g) { return g.inverse();}
};

/// Both multiplicative group traits and Testable
template<class Class>
struct MultiplicativeGroup : MultiplicativeGroupTraits<Class>, Testable<Class> {};

/// A helper class that implements the traits interface for additive groups.
/// Assumes existence of identity and three additive operators
template<class Class>
struct AdditiveGroupTraits {
  typedef group_tag structure_category;
  typedef additive_group_tag group_flavor;
  static Class Identity() { return Class::Identity(); }
  static Class Compose(const Class &g, const Class & h) { return g + h;}
  static Class Between(const Class &g, const Class & h) { return h - g;}
  static Class Inverse(const Class &g) { return -g;}
};

/// Both additive group traits and Testable
template<class Class>
struct AdditiveGroup : AdditiveGroupTraits<Class>, Testable<Class> {};

}  // namespace internal

/// compose multiple times
#ifdef USING_BOOST_DEV
template<typename G>
BOOST_CONCEPT_REQUIRES(((IsGroup<G>)),(G)) //
compose_pow(const G& g, size_t n) {
  if (n == 0) return traits<G>::Identity();
  else if (n == 1) return g;
  else return traits<G>::Compose(compose_pow(g, n - 1), g);
}
#else
// Now we can define the compose_pow function template using std::enable_if to
// ensure that G satisfies the IsGroup concept.

template<typename G>
typename std::enable_if<is_group<G>::value, G>::type
compose_pow(const G& g, size_t n) {
  if (n == 0) return traits<G>::Identity();
  else if (n == 1) return g;
  else return traits<G>::Compose(compose_pow(g, n - 1), g);
}
#endif

/// Template to construct the direct product of two arbitrary groups
/// Assumes nothing except group structure and Testable from G and H
template<typename G, typename H>
class DirectProduct: public std::pair<G, H> {
  #ifdef USING_BOOST_DEV
  BOOST_CONCEPT_ASSERT((IsGroup<G>));
  BOOST_CONCEPT_ASSERT((IsGroup<H>));
  #endif

public:
  /// Default constructor yields identity
  DirectProduct():std::pair<G,H>(traits<G>::Identity(),traits<H>::Identity()) {}

  // Construct from two subgroup elements
  DirectProduct(const G& g, const H& h):std::pair<G,H>(g,h) {}

  // identity
  static DirectProduct Identity() { return DirectProduct(); }

  DirectProduct operator*(const DirectProduct& other) const {
    return DirectProduct(traits<G>::Compose(this->first, other.first),
        traits<H>::Compose(this->second, other.second));
  }
  DirectProduct inverse() const {
    return DirectProduct(this->first.inverse(), this->second.inverse());
  }
};

// Define any direct product group to be a model of the multiplicative Group concept
template<typename G, typename H>
struct traits<DirectProduct<G, H> > :
  internal::MultiplicativeGroupTraits<DirectProduct<G, H> > {};

/// Template to construct the direct sum of two additive groups
/// Assumes existence of three additive operators for both groups
template<typename G, typename H>
class DirectSum: public std::pair<G, H> {
  #ifdef USING_BOOST_DEV
  BOOST_CONCEPT_ASSERT((IsGroup<G>));
  BOOST_CONCEPT_ASSERT((IsGroup<H>));
  #endif

  const G& g() const { return this->first; }
  const H& h() const { return this->second;}

public:
  /// Default constructor yields identity
  DirectSum():std::pair<G,H>(traits<G>::Identity(),traits<H>::Identity()) {}

  // Construct from two subgroup elements
  DirectSum(const G& g, const H& h):std::pair<G,H>(g,h) {}

  // identity
  static DirectSum Identity() { return DirectSum(); }

  DirectSum operator+(const DirectSum& other) const {
    return DirectSum(g()+other.g(), h()+other.h());
  }
  DirectSum operator-(const DirectSum& other) const {
    return DirectSum(g()-other.g(), h()-other.h());
  }
  DirectSum operator-() const {
    return DirectSum(- g(), - h());
  }
};

// Define direct sums to be a model of the Additive Group concept
template<typename G, typename H>
struct traits<DirectSum<G, H> > :
  internal::AdditiveGroupTraits<DirectSum<G, H> > {};

}  // namespace gtsam

/**
 * Macros for using the IsGroup
 *  - An instantiation for use inside unit tests
 *  - A typedef for use inside generic algorithms
 *
 * NOTE: intentionally not in the gtsam namespace to allow for classes not in
 * the gtsam namespace to be more easily enforced as testable
 */
#define GTSAM_CONCEPT_GROUP_INST(T) template class gtsam::IsGroup<T>;
#define GTSAM_CONCEPT_GROUP_TYPE(T) typedef gtsam::IsGroup<T> _gtsam_IsGroup_##T;
