#include "dimensions.hpp"

namespace // TODO looks terrible
{

#ifndef PI
constexpr double PI = 3.14159265359;
#endif

#ifndef RADIAN
constexpr double RADIAN = 57.2957795131;
#endif

} // namespace

namespace kot_motor::dimensions {

namespace literals {

// fundamental dimensions
Meter operator""_m(long double val)
{
  return val;
}

Meter operator""_m(unsigned long long int val)
{
  return val;
}

Kg operator""_kg(long double val)
{
  return val;
}

Kg operator""_kg(unsigned long long int val)
{
  return val;
}

Second operator""_s(long double val)
{
  return val;
}

Second operator""_s(unsigned long long int val)
{
  return val;
}

Amp operator""_A(long double val)
{
  return val;
}

Amp operator""_A(unsigned long long int val)
{
  return val;
}

Kelvin operator""_K(long double val)
{
  return val;
}

Kelvin operator""_K(unsigned long long int val)
{
  return val;
}

Candela operator""_cd(long double val)
{
  return val;
}

Candela operator""_cd(unsigned long long int val)
{
  return val;
}

Mol operator""_mol(long double val)
{
  return val;
}

Mol operator""_mol(unsigned long long int val)
{
  return val;
}

Rad operator""_rad(long double val)
{
  return val;
}

Rad operator""_rad(unsigned long long int val)
{
  return val;
}

Deg operator""_deg(long double val)
{
  return val;
}

Deg operator""_deg(unsigned long long int val)
{
  return val;
}

// derived measurements
Velocity operator""_m_per_s(long double val)
{
  return val;
}

Velocity operator""_m_per_s(unsigned long long int val)
{
  return val;
}

Accel operator""_m_per_s2(long double val)
{
  return val;
}

Accel operator""_m_per_s2(unsigned long long int val)
{
  return val;
}

Newton operator""_N(long double val)
{
  return val;
}

Newton operator""_N(unsigned long long int val)
{
  return val;
}

Joule operator""_J(long double val)
{
  return val;
}

Joule operator""_J(unsigned long long int val)
{
  return val;
}

Joule operator""_joule(long double val)
{
  return val;
}

Joule operator""_joule(unsigned long long int val)
{
  return val;
}

// alternative naming
Percent operator""_percent(long double val)
{
  return val;
}

Percent operator""_percent(unsigned long long int val)
{
  return val;
}

Percent operator""_pct(long double val)
{
  return val;
}

Percent operator""_pct(unsigned long long int val)
{
  return val;
}

Length operator""_length(long double val)
{
  return val;
}

Length operator""_length(unsigned long long int val)
{
  return val;
}

Mass operator""_mass(long double val)
{
  return val;
}

Mass operator""_mass(unsigned long long int val)
{
  return val;
}

Time operator""_time(long double val)
{
  return val;
}

Time operator""_time(unsigned long long int val)
{
  return val;
}

Ampere operator""_amps(long double val)
{
  return val;
}

Ampere operator""_amps(unsigned long long int val)
{
  return val;
}

Current operator""_current(long double val)
{
  return val;
}

Current operator""_current(unsigned long long int val)
{
  return val;
}

Temperature operator""_temp(long double val)
{
  return val;
}

Temperature operator""_temp(unsigned long long int val)
{
  return val;
}

Light operator""_light(long double val)
{
  return val;
}

Light operator""_light(unsigned long long int val)
{
  return val;
}

Concentration operator""_concentration(long double val)
{
  return val;
}

Concentration operator""_concentration(unsigned long long int val)
{
  return val;
}

Radian operator""_radians(long double val)
{
  return val;
}

Radian operator""_radians(unsigned long long int val)
{
  return val;
}

Degree operator""_degree(long double val)
{
  return val;
}

Degree operator""_degree(unsigned long long int val)
{
  return val;
}

Energy operator""_energy(long double val)
{
  return val;
}

Energy operator""_energy(unsigned long long int val)
{
  return val;
}

NewtonMeter operator""_Nm(long double val)
{
  return val;
}

NewtonMeter operator""_Nm(unsigned long long int val)
{
  return val;
}

Torque operator""_torq(long double val)
{
  return val;
}

Torque operator""_torq(unsigned long long int val)
{
  return val;
}

Force operator""_force(long double val)
{
  return val;
}

Force operator""_force(unsigned long long int val)
{
  return val;
}

TranslationalStiffness operator""_ts(long double val)
{
  return val;
}

TranslationalStiffness operator""_ts(unsigned long long int val)
{
  return val;
}

RotationalStiffness operator""_rs(long double val)
{
  return val;
}

RotationalStiffness operator""_rs(unsigned long long int val)
{
  return val;
}

TranslationalDamping operator""_td(long double val)
{
  return val;
}

TranslationalDamping operator""_td(unsigned long long int val)
{
  return val;
}

RotationalDamping operator""_rd(long double val)
{
  return val;
}

RotationalDamping operator""_rd(unsigned long long int val)
{
  return val;
}

AngularVelocity operator""_av(long double val)
{
  return val;
}

AngularVelocity operator""_av(unsigned long long int val)
{
  return val;
}

Work operator""_work(long double val)
{
  return val;
}

Work operator""_work(unsigned long long int val)
{
  return val;
}

Voltage operator""_voltage(long double val)
{
  return val;
}

Voltage operator""_voltage(unsigned long long int val)
{
  return val;
}

Resistance operator""_resistance(long double val)
{
  return val;
}

Resistance operator""_resistance(unsigned long long int val)
{
  return val;
}

Watt operator""_watt(long double val)
{
  return val;
}

Watt operator""_watt(unsigned long long int val)
{
  return val;
}

Volt operator""_volt(long double val)
{
  return val;
}

Volt operator""_volt(unsigned long long int val)
{
  return val;
}

Ohm operator""_ohm(long double val)
{
  return val;
}

Ohm operator""_ohm(unsigned long long int val)
{
  return val;
}

Weight operator""_weight(long double val)
{
  return val;
}

Weight operator""_weight(unsigned long long int val)
{
  return val;
}
} // namespace litrals

Degree rad_to_deg(const Radian & rad)
{
  return float(rad) * RADIAN;
}

Radian deg_to_rad(const Degree & deg)
{
  return float(deg) / RADIAN;
}

} // namespace kot_motor::dimensions




