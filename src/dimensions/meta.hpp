#ifndef META_HPP
#define META_HPP

#include <stddef.h>

namespace kot_motor::dimensions::meta {

template <int a, int b>
struct Plus {
  static int const value = a + b;
};

template <int a, int b>
struct Minus {
  static int const value = a - b;
};

template <int... Types>
struct IntList;

template <int H, int... T>
struct IntList<H, T...> {
  const static int Head = H;
  using Tail = IntList<T...>;
};

template <>
struct IntList<> { };

template <typename IL>
struct CountLength {
  static size_t const value = 1 + CountLength<typename IL::Tail>::value;
};

template <>
struct CountLength<IntList<>> {
  const static size_t value = 0;
};

template <int N, typename IL>
struct IntCons;

template <int N, int... Ns>
struct IntCons<N, IntList<Ns...>> {
  using type = IntList<N, Ns...>;
};

template <size_t Size, typename IL = IntList<>>
struct Generate {
  using type =
    typename Generate<Size - 1, typename IntCons<Size - 1, IL>::type>::type;
};

template <typename IL>
struct Generate<0, IL> {
  using type = IL;
};

template <typename IL1, typename IL2, template <int, int> class Func>
struct Transform;

template <int... IL1s, int... IL2s, template <int, int> class Func>
struct Transform<IntList<IL1s...>, IntList<IL2s...>, Func> {
  using type = IntList<Func<IL1s, IL2s>::value...>;
};

} // namespace kot_motor::dimensions::meta

#endif // META_HPP




