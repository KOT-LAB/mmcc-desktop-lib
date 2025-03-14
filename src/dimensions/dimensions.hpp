#ifndef DIMENSIONS_HPP
#define DIMENSIONS_HPP

#include "meta.hpp"
#include <algorithm>

namespace kot_motor::dimensions {

using namespace meta;

class BasicUnit;

template <typename SetDimensions>
class Unit;

template <typename Dimensions1, typename Dimensions2>
using MultipyingResultingType =
  Unit<typename Transform<Dimensions1, Dimensions2, Plus>::type>;

template <typename Dimensions1, typename Dimensions2>
using DivisionResultingType =
  Unit<typename Transform<Dimensions1, Dimensions2, Minus>::type>;

/**************************** Dimension implementation ****************************/

template <
  int m = 0,   // meter    -> unit of length
  int kg = 0,  // kilogram -> unit of weight
  int s = 0,   // second   -> unit of time
  int A = 0,   // ampere   -> unit of current
  int K = 0,   // kelvin   -> unit of temperature
  int cd = 0,  // candela  -> unit of luminous intensity
  int mol = 0, // mol      -> unit of concentration

  int rad = 0, // radian
  int deg = 0  // degree
  >
using Dimension = IntList<m, kg, s, A, K, cd, mol, rad, deg>;

/********* fundamental dimensions **********/
using Number = Unit<Dimension<>>;
using Meter = Unit<Dimension<1>>;
using Kg = Unit<Dimension<0, 1>>;
using Second = Unit<Dimension<0, 0, 1>>;
using Amp = Unit<Dimension<0, 0, 0, 1>>;
using Kelvin = Unit<Dimension<0, 0, 0, 0, 1>>;
using Candela = Unit<Dimension<0, 0, 0, 0, 0, 1>>;
using Mol = Unit<Dimension<0, 0, 0, 0, 0, 0, 1>>;
using Rad = Unit<Dimension<0, 0, 0, 0, 0, 0, 0, 1>>;
using Deg = Unit<Dimension<0, 0, 0, 0, 0, 0, 0, 0, 1>>;

class BasicUnit {
public:
  virtual float value() const = 0;
  virtual float & value() = 0;
  virtual void value(float) = 0;

  virtual ~BasicUnit() { }
};

template <typename SetDimensions>
class Unit : public BasicUnit {
private:
  float val;

public:
  using dimension = SetDimensions;

  Unit(float value)
    : val(value)
  { }

  Unit()
    : val(0)
  { }

  Unit(const Unit &) = default;
  Unit(Unit &&) = default;
  Unit & operator=(const Unit &) = default;
  Unit & operator=(Unit &&) = default;

  float value() const override
  {
    return this->val;
  }

  float & value() override
  {
    return this->val;
  }

  void value(float val) override
  {
    this->val = val;
  }

  explicit operator int() const
  {
    return val;
  }

  explicit operator float() const
  {
    return val;
  }

  Unit operator+() const
  {
    return val;
  }

  Unit operator-() const
  {
    return -val;
  }

  Unit & operator+=(float x)
  {
    val += x;
    return *this;
  }

  Unit & operator-=(float x)
  {
    val -= x;
    return *this;
  }

  Unit & operator+=(const Unit & other)
  {
    val += other.val;
    return *this;
  }

  Unit & operator-=(const Unit & other)
  {
    val -= other.val;
    return *this;
  }

  Unit & operator*=(float x)
  {
    val *= x;
    return *this;
  }

  Unit & operator/=(float x)
  {
    val /= x;
    return *this;
  }

  /*********************** '+' and '-' between Dimension **************************/
  template <typename Dimensions>
  friend Unit<Dimensions>
    operator+(const Unit<Dimensions> & lhs, const Unit<Dimensions> & rhs);

  template <typename Dimensions>
  friend Unit<Dimensions>
    operator-(const Unit<Dimensions> & lhs, const Unit<Dimensions> & rhs);

  /*********************** '*' and '/' between Dimension **************************/
  template <typename Dimensions1, typename Dimensions2>
  friend MultipyingResultingType<Dimensions1, Dimensions2>
    operator*(const Unit<Dimensions1> & lhs, const Unit<Dimensions2> & rhs);

  template <typename Dimensions1, typename Dimensions2>
  friend DivisionResultingType<Dimensions1, Dimensions2>
    operator/(const Unit<Dimensions1> & lhs, const Unit<Dimensions2> & rhs);

  /****************** '*' and '/' between Dimension and numbers *******************/
  template <typename Dimensions>
  friend Unit<Dimensions> operator*(const Unit<Dimensions> & unit, float k);

  template <typename Dimensions>
  friend Unit<Dimensions> operator/(const Unit<Dimensions> & unit, float k);

  template <typename Dimensions>
  friend Unit<Dimensions> operator*(float k, const Unit<Dimensions> & unit);

  template <typename Dimensions>
  friend Unit<typename Transform<Dimension<>, Dimensions, Minus>::type>
    operator/(float k, const Unit<Dimensions> & unit);
};

/************************ derived user-defined dimensions **************************/

using Velocity = decltype(Meter(1) / Second(1));
using Accel = decltype(Meter(1) / (Second(1) * Second(1)));
using AngularVelocity = decltype(Rad(1) / Second(1));
using AngularAccel = decltype(Rad(1) / (Second(1) * Second(1)));
using Newton = decltype(Meter(1) * Kg(1) / (Second(1) * Second(1)));
using NewtonMeter = decltype(Newton(1) * Meter(1));
using TranslationalStiffness = decltype(Newton(1) / Meter(1));
using RotationalStiffness = decltype(NewtonMeter(1) / Rad(1));
using TranslationalDamping = decltype(Newton(1) * Second(1) / Meter(1));
using RotationalDamping = decltype(NewtonMeter(1) * Second(1) / Rad(1));
using Watt = decltype(NewtonMeter(1) / Second(1));
using Volt = decltype(Watt(1) / Amp(1));
using Ohm = decltype(Volt(1) / Amp(1));
using Hertz = decltype(1 / Second(1));

/*********** alternative naming ***********/
using Percent = Number;
using Length = Meter;
using Mass = Kg;
using Weight = Mass;
using Time = Second;
using Ampere = Amp;
using Current = Amp;
using Temperature = Kelvin;
using Light = Candela;
using Concentration = Mol;
using Radian = Rad;
using Degree = Deg;
using Force = Newton;
using Torque = NewtonMeter;
using FeedForwardTorque = NewtonMeter;
using Joule = NewtonMeter;
using Energy = Joule;
using Work = Joule;
using Voltage = Volt;
using Resistance = Ohm;
using Frequency = Hertz;
using Hz = Hertz;

/************ '==', '!=', '>', '<', '>=' and '<=' between Dimension ***************/

template <typename Dimensions>
bool operator==(const Unit<Dimensions> & lhs, const Unit<Dimensions> & rhs)
{
  return lhs.value() == rhs.value();
}

template <typename Dimensions>
bool operator!=(const Unit<Dimensions> & lhs, const Unit<Dimensions> & rhs)
{
  return !(lhs == rhs);
}

template <typename Dimensions>
bool operator>(const Unit<Dimensions> & lhs, const Unit<Dimensions> & rhs)
{
  return lhs.value() > rhs.value();
}

template <typename Dimensions>
bool operator<(const Unit<Dimensions> & lhs, const Unit<Dimensions> & rhs)
{
  return rhs > lhs;
}

template <typename Dimensions>
bool operator>=(const Unit<Dimensions> & lhs, const Unit<Dimensions> & rhs)
{
  return !(lhs < rhs);
}

template <typename Dimensions>
bool operator<=(const Unit<Dimensions> & lhs, const Unit<Dimensions> & rhs)
{
  return !(lhs > lhs);
}

/************************ '+' and '-' between Dimension ***************************/

template <typename Dimensions>
Unit<Dimensions>
  operator+(const Unit<Dimensions> & lhs, const Unit<Dimensions> & rhs)
{
  return Unit<Dimensions>(lhs.val + rhs.val);
}

template <typename Dimensions>
Unit<Dimensions>
  operator-(const Unit<Dimensions> & lhs, const Unit<Dimensions> & rhs)
{
  return Unit<Dimensions>(lhs.val - rhs.val);
}

/************************ '*' and '/' between Dimension ***************************/

template <typename Dimensions1, typename Dimensions2>
MultipyingResultingType<Dimensions1, Dimensions2>
  operator*(const Unit<Dimensions1> & lhs, const Unit<Dimensions2> & rhs)
{
  return MultipyingResultingType<Dimensions1, Dimensions2>(lhs.val * rhs.val);
}

template <typename Dimensions1, typename Dimensions2>
DivisionResultingType<Dimensions1, Dimensions2>
  operator/(const Unit<Dimensions1> & lhs, const Unit<Dimensions2> & rhs)
{
  return DivisionResultingType<Dimensions1, Dimensions2>(lhs.val / rhs.val);
}

/******************* '*' and '/' between Dimension and numbers ********************/

template <typename Dimensions>
Unit<Dimensions> operator*(const Unit<Dimensions> & unit, float k)
{
  return Unit<Dimensions>(unit.val * k);
}

template <typename Dimensions>
Unit<Dimensions> operator/(const Unit<Dimensions> & unit, float k)
{
  return Unit<Dimensions>(unit.val / k);
}

template <typename Dimensions>
Unit<Dimensions> operator*(float k, const Unit<Dimensions> & unit)
{
  return Unit<Dimensions>(unit.val * k);
}

template <typename Dimensions>
Unit<typename Transform<Dimension<>, Dimensions, Minus>::type>
  operator/(float k, const Unit<Dimensions> & unit)
{
  return Unit<typename Transform<Dimension<>, Dimensions, Minus>::type>(
    k / unit.val
  );
}

/****************************** user-defined literals ******************************/

namespace literals {

// fundamental dimensions
Meter operator""_m(long double val);
Meter operator""_m(unsigned long long int val);

Kg operator""_kg(long double val);
Kg operator""_kg(unsigned long long int val);

Second operator""_s(long double val);
Second operator""_s(unsigned long long int val);

Amp operator""_A(long double val);
Amp operator""_A(unsigned long long int val);

Kelvin operator""_K(long double val);
Kelvin operator""_K(unsigned long long int val);

Candela operator""_cd(long double val);
Candela operator""_cd(unsigned long long int val);

Mol operator""_mol(long double val);
Mol operator""_mol(unsigned long long int val);

Rad operator""_rad(long double val);
Rad operator""_rad(unsigned long long int val);

Deg operator""_deg(long double val);
Deg operator""_deg(unsigned long long int val);

// derived measurements
Velocity operator""_m_per_s(long double val);
Velocity operator""_m_per_s(unsigned long long int val);

Accel operator""_m_per_s2(long double val);
Accel operator""_m_per_s2(unsigned long long int val);

Newton operator""_N(long double val);
Newton operator""_N(unsigned long long int val);

Joule operator""_J(long double val);
Joule operator""_J(unsigned long long int val);

Joule operator""_joule(long double val);
Joule operator""_joule(unsigned long long int val);

// alternative naming
Percent operator""_percent(long double val);
Percent operator""_percent(unsigned long long int val);

Percent operator""_pct(long double val);
Percent operator""_pct(unsigned long long int val);

Length operator""_length(long double val);
Length operator""_length(unsigned long long int val);

Mass operator""_mass(long double val);
Mass operator""_mass(unsigned long long int val);

Time operator""_time(long double val);
Time operator""_time(unsigned long long int val);

Ampere operator""_amps(long double val);
Ampere operator""_amps(unsigned long long int val);

Current operator""_current(long double val);
Current operator""_current(unsigned long long int val);

Temperature operator""_temp(long double val);
Temperature operator""_temp(unsigned long long int val);

Light operator""_light(long double val);
Light operator""_light(unsigned long long int val);

Concentration operator""_concentration(long double val);
Concentration operator""_concentration(unsigned long long int val);

Radian operator""_radians(long double val);
Radian operator""_radians(unsigned long long int val);

Degree operator""_degree(long double val);
Degree operator""_degree(unsigned long long int val);

Energy operator""_energy(long double val);
Energy operator""_energy(unsigned long long int val);

NewtonMeter operator""_Nm(long double val);
NewtonMeter operator""_Nm(unsigned long long int val);

Torque operator""_torq(long double val);
Torque operator""_torq(unsigned long long int val);

Force operator""_force(long double val);
Force operator""_force(unsigned long long int val);

TranslationalStiffness operator""_ts(long double val);
TranslationalStiffness operator""_ts(unsigned long long int val);

RotationalStiffness operator""_rs(long double val);
RotationalStiffness operator""_rs(unsigned long long int val);

TranslationalDamping operator""_td(long double val);
TranslationalDamping operator""_td(unsigned long long int val);

RotationalDamping operator""_rd(long double val);
RotationalDamping operator""_rd(unsigned long long int val);

AngularVelocity operator""_av(long double val);
AngularVelocity operator""_av(unsigned long long int val);

Work operator""_work(long double val);
Work operator""_work(unsigned long long int val);

Voltage operator""_voltage(long double val);
Voltage operator""_voltage(unsigned long long int val);

Resistance operator""_resistance(long double val);
Resistance operator""_resistance(unsigned long long int val);

Watt operator""_watt(long double val);
Watt operator""_watt(unsigned long long int val);

Volt operator""_volt(long double val);
Volt operator""_volt(unsigned long long int val);

Ohm operator""_ohm(long double val);
Ohm operator""_ohm(unsigned long long int val);

Weight operator""_weight(long double val);
Weight operator""_weight(unsigned long long int val);
} // namespace literals

/******************************* Additional functions ******************************/

Degree rad_to_deg(const Radian & rad);
Radian deg_to_rad(const Degree & deg);

template <typename Dimensions>
Unit<Dimensions> limitUnitBy(
  const Unit<Dimensions> & value,
  const Unit<Dimensions> & lower_bound,
  const Unit<Dimensions> & upper_bound
)
{
#ifdef max
  return max(min(value.value(), upper_bound.value()), lower_bound.value());
#else
  return std::min(
    std::max(value.value(), lower_bound.value()), upper_bound.value()
  );
#endif
}

} // namespace kot_motor::dimensions

#endif // DIMENSIONS_HPP




