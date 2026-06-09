#pragma once

#define NEW_UNIT_LITERAL(QuantityType, short_name, long_name, UnitType, ScaleNum, ScaleDen) \
  struct UnitType {                                                                         \
    typedef typename dimension_for_quantity<QuantityType>::type dimension;                  \
    static constexpr double scale_num = static_cast<double>(ScaleNum);                      \
    static constexpr double scale_den = static_cast<double>(ScaleDen);                      \
    static constexpr bool is_affine = false;                                                \
    static const char* symbol() { return #short_name; }                                     \
    static const char* name() { return #long_name; }                                        \
  };                                                                                        \
  namespace literals {                                                                      \
  constexpr QuantityType operator""_##short_name(long double value) {                       \
    return QuantityType::from<UnitType>(static_cast<double>(value));                        \
  }                                                                                         \
  constexpr QuantityType operator""_##short_name(unsigned long long value) {                \
    return QuantityType::from<UnitType>(static_cast<double>(value));                        \
  }                                                                                         \
  }

#define NEW_ALIAS_UNIT_LITERAL(QuantityType, short_name, long_name, UnitType, ...)

#define NEW_UNIT(QuantityType, DimType, M, L, Ti, I, Th, N, J, A, DefaultUnitTag, IsCanonicalResult) \
  typedef dim<M, L, Ti, I, Th, N, J, A> DimType;                                                     \
  struct DefaultUnitTag;                                                                             \
  class QuantityType;                                                                                \
  template <>                                                                                        \
  struct dimension_for_quantity<QuantityType> {                                                      \
    typedef DimType type;                                                                            \
  };                                                                                                 \
  template <>                                                                                        \
  struct is_quantity<QuantityType> : std::true_type {};                                              \
  template <bool Enabled = IsCanonicalResult, typename std::enable_if<Enabled, int>::type = 0>       \
  QuantityType quantity_for_dimension_probe(DimType*, int);                                          \
  template <>                                                                                        \
  struct default_unit_for_quantity<QuantityType> {                                                   \
    typedef DefaultUnitTag type;                                                                     \
  };                                                                                                 \
  class QuantityType : public quantity_base<QuantityType, DimType> {                                 \
   public:                                                                                           \
    typedef quantity_base<QuantityType, DimType> base_type;                                          \
    typedef base_type semantic_base_type;                                                            \
    using base_type::base_type;                                                                      \
  };

NEW_UNIT(Amount, dim_amount, 0, 0, 0, 0, 0, 1, 0, 0, mole_t, true)
NEW_UNIT_LITERAL(Amount, mol, moles, mole_t, 1, 1)

NEW_UNIT(Current, dim_current, 0, 0, 0, 1, 0, 0, 0, 0, ampere_t, true)
NEW_UNIT_LITERAL(Current, A, amperes, ampere_t, 1, 1)
NEW_UNIT_LITERAL(Current, mA, milliamperes, milliampere_t, 1, 1000)
NEW_UNIT_LITERAL(Current, uA, microamperes, microampere_t, 1, 1000000)

NEW_UNIT(Length, dim_length, 0, 1, 0, 0, 0, 0, 0, 0, meter_t, true)
NEW_UNIT_LITERAL(Length, m, meters, meter_t, 1, 1)
NEW_UNIT_LITERAL(Length, mm, millimeters, millimeter_t, 1, 1000)
NEW_UNIT_LITERAL(Length, cm, centimeters, centimeter_t, 1, 100)
NEW_UNIT_LITERAL(Length, km, kilometers, kilometer_t, 1000, 1)
NEW_UNIT_LITERAL(Length, in, inches, inch_t, 127, 5000)
NEW_UNIT_LITERAL(Length, tile, tiles, tile_t, 2413, 4000)
NEW_UNIT_LITERAL(Length, ft, feet, foot_t, 381, 1250)
NEW_UNIT_LITERAL(Length, yd, yards, yard_t, 1143, 1250)
NEW_UNIT_LITERAL(Length, mi, miles, mile_t, 201168, 125)

NEW_UNIT(LuminousIntensity, dim_luminous_intensity, 0, 0, 0, 0, 0, 0, 1, 0, candela_t, true)
NEW_UNIT_LITERAL(LuminousIntensity, cd, candelas, candela_t, 1, 1)

NEW_UNIT(Mass, dim_mass, 1, 0, 0, 0, 0, 0, 0, 0, kilogram_t, true)
NEW_UNIT_LITERAL(Mass, kg, kilograms, kilogram_t, 1, 1)
NEW_UNIT_LITERAL(Mass, g, grams, gram_t, 1, 1000)
NEW_UNIT_LITERAL(Mass, oz, ounces, ounce_t, 28349523125LL, 1000000000000LL)
NEW_UNIT_LITERAL(Mass, lb, pounds, pound_t, 45359237, 100000000)
NEW_UNIT_LITERAL(Mass, ton, tons, ton_t, 90718474, 100000)

NEW_UNIT(Time, dim_time, 0, 0, 1, 0, 0, 0, 0, 0, second_t, true)
NEW_UNIT_LITERAL(Time, s, seconds, second_t, 1, 1)
NEW_UNIT_LITERAL(Time, ms, milliseconds, millisecond_t, 1, 1000)
NEW_UNIT_LITERAL(Time, us, microseconds, microsecond_t, 1, 1000000)
NEW_UNIT_LITERAL(Time, min, minutes, minute_t, 60, 1)
NEW_UNIT_LITERAL(Time, hr, hours, hour_t, 3600, 1)
NEW_UNIT_LITERAL(Time, day, days, day_t, 86400, 1)

NEW_UNIT(Area, dim_area, 0, 2, 0, 0, 0, 0, 0, 0, square_meter_t, true)
NEW_UNIT_LITERAL(Area, m2, square_meters, square_meter_t, 1, 1)
NEW_UNIT_LITERAL(Area, ft2, square_feet, square_foot_t, 145161, 1562500)
NEW_UNIT_LITERAL(Area, yd2, square_yards, square_yard_t, 10451592, 12500000)
NEW_UNIT_LITERAL(Area, acre, acres, acre_t, 316160658, 78125)

NEW_UNIT(Volume, dim_volume, 0, 3, 0, 0, 0, 0, 0, 0, cubic_meter_t, true)
NEW_UNIT_LITERAL(Volume, m3, cubic_meters, cubic_meter_t, 1, 1)
NEW_UNIT_LITERAL(Volume, in3, cubic_inches, cubic_inch_t, 2048383, 125000000000)
NEW_UNIT_LITERAL(Volume, ft3, cubic_feet, cubic_foot_t, 55306341, 1953125000)
NEW_UNIT_LITERAL(Volume, liter, liters, liter_t, 1, 1000)
NEW_UNIT_LITERAL(Volume, gal, gallons, us_gallon_t, 473176473, 125000000000)
NEW_UNIT_LITERAL(Volume, qt, quarts, us_quart_t, 473176473, 500000000000)
NEW_UNIT_LITERAL(Volume, pt, pints, us_pint_t, 473176473, 1000000000000)
NEW_UNIT_LITERAL(Volume, floz, fluid_ounces, us_fluid_ounce_t, 473176473, 16000000000000)

NEW_UNIT(Velocity, dim_velocity, 0, 1, -1, 0, 0, 0, 0, 0, meter_per_second_t, true)
NEW_UNIT_LITERAL(Velocity, mps, meters_per_second, meter_per_second_t, 1, 1)
NEW_UNIT_LITERAL(Velocity, inps, inches_per_second, inch_per_second_t, 127, 5000)
NEW_UNIT_LITERAL(Velocity, ftps, feet_per_second, foot_per_second_t, 381, 1250)
NEW_UNIT_LITERAL(Velocity, mph, miles_per_hour, mile_per_hour_t, 1397, 3125)

NEW_UNIT(Acceleration, dim_acceleration, 0, 1, -2, 0, 0, 0, 0, 0, meter_per_second_squared_t, true)
NEW_UNIT_LITERAL(Acceleration, mps2, meters_per_second_squared, meter_per_second_squared_t, 1, 1)
NEW_UNIT_LITERAL(Acceleration, inps2, inches_per_second_squared, inch_per_second_squared_t, 127, 5000)

NEW_UNIT(Jerk, dim_jerk, 0, 1, -3, 0, 0, 0, 0, 0, meter_per_second_cubed_t, true)
NEW_UNIT_LITERAL(Jerk, mps3, meters_per_second_cubed, meter_per_second_cubed_t, 1, 1)
NEW_UNIT_LITERAL(Jerk, inps3, inches_per_second_cubed, inch_per_second_cubed_t, 127, 5000)

NEW_UNIT(Absement, dim_absement, 0, 1, 1, 0, 0, 0, 0, 0, meter_second_t, true)
NEW_UNIT_LITERAL(Absement, m_s, meter_seconds, meter_second_t, 1, 1)
NEW_UNIT_LITERAL(Absement, in_s, inch_seconds, inch_second_t, 127, 5000)

NEW_UNIT(AngularVelocity, dim_angular_velocity, 0, 0, -1, 0, 0, 0, 0, 1, radian_per_second_t, true)
NEW_UNIT_LITERAL(AngularVelocity, radps, radians_per_second, radian_per_second_t, 1, 1)
NEW_UNIT_LITERAL(AngularVelocity, degps, degrees_per_second, degree_per_second_t, 17453292519943295.0,
                 1000000000000000000.0)
NEW_UNIT_LITERAL(AngularVelocity, rpm, revolutions_per_minute, revolution_per_minute_t, 10471975511965977.0,
                 100000000000000000.0)

NEW_UNIT(AngularAcceleration, dim_angular_acceleration, 0, 0, -2, 0, 0, 0, 0, 1, radian_per_second_squared_t, true)
NEW_UNIT_LITERAL(AngularAcceleration, radps2, radians_per_second_squared, radian_per_second_squared_t, 1, 1)
NEW_UNIT_LITERAL(AngularAcceleration, degps2, degrees_per_second_squared, degree_per_second_squared_t,
                 17453292519943295.0, 1000000000000000000.0)

NEW_UNIT(AngularJerk, dim_angular_jerk, 0, 0, -3, 0, 0, 0, 0, 1, radian_per_second_cubed_t, true)
NEW_UNIT_LITERAL(AngularJerk, radps3, radians_per_second_cubed, radian_per_second_cubed_t, 1, 1)
NEW_UNIT_LITERAL(AngularJerk, degps3, degrees_per_second_cubed, degree_per_second_cubed_t, 17453292519943295.0,
                 1000000000000000000.0)

NEW_UNIT(AngularAbsement, dim_angular_absement, 0, 0, 1, 0, 0, 0, 0, 1, radian_second_t, true)
NEW_UNIT_LITERAL(AngularAbsement, rad_s, radian_seconds, radian_second_t, 1, 1)
NEW_UNIT_LITERAL(AngularAbsement, deg_s, degree_seconds, degree_second_t, 17453292519943295.0, 1000000000000000000.0)

NEW_UNIT(Curvature, dim_curvature, 0, -1, 0, 0, 0, 0, 0, 1, radian_per_meter_t, true)
NEW_UNIT_LITERAL(Curvature, radpm, radians_per_meter, radian_per_meter_t, 1, 1)
NEW_UNIT_LITERAL(Curvature, degpm, degrees_per_meter, degree_per_meter_t, 17453292519943295.0, 1000000000000000000.0)
NEW_UNIT_LITERAL(Curvature, radpft, radians_per_foot, radian_per_foot_t, 1250, 381)
NEW_UNIT_LITERAL(Curvature, degpft, degrees_per_foot, degree_per_foot_t, 21816615649929118.0, 290322580645161280.0)

NEW_UNIT(Frequency, dim_frequency, 0, 0, -1, 0, 0, 0, 0, 0, hertz_t, true)
NEW_UNIT_LITERAL(Frequency, Hz, hertz, hertz_t, 1, 1)

NEW_UNIT(Charge, dim_charge, 0, 0, 1, 1, 0, 0, 0, 0, coulomb_t, true)
NEW_UNIT_LITERAL(Charge, coul, coulombs, coulomb_t, 1, 1)

NEW_UNIT(Voltage, dim_voltage, 1, 2, -3, -1, 0, 0, 0, 0, volt_t, true)
NEW_UNIT_LITERAL(Voltage, V, volts, volt_t, 1, 1)
NEW_UNIT_LITERAL(Voltage, mV, millivolts, millivolt_t, 1, 1000)

NEW_UNIT(Resistance, dim_resistance, 1, 2, -3, -2, 0, 0, 0, 0, ohm_t, true)
NEW_UNIT_LITERAL(Resistance, ohm, ohms, ohm_t, 1, 1)
NEW_UNIT_LITERAL(Resistance, kohm, kiloohms, kiloohm_t, 1000, 1)

NEW_UNIT(Conductance, dim_conductance, -1, -2, 3, 2, 0, 0, 0, 0, siemens_t, true)
NEW_UNIT_LITERAL(Conductance, siem, siemens, siemens_t, 1, 1)

NEW_UNIT(Capacitance, dim_capacitance, -1, -2, 4, 2, 0, 0, 0, 0, farad_t, true)
NEW_UNIT_LITERAL(Capacitance, F, farads, farad_t, 1, 1)
NEW_UNIT_LITERAL(Capacitance, mF, millifarads, millifarad_t, 1, 1000)
NEW_UNIT_LITERAL(Capacitance, uF, microfarads, microfarad_t, 1, 1000000)

NEW_UNIT(Inductance, dim_inductance, 1, 2, -2, -2, 0, 0, 0, 0, henry_t, true)
NEW_UNIT_LITERAL(Inductance, H, henries, henry_t, 1, 1)

NEW_UNIT(MagneticFlux, dim_magnetic_flux, 1, 2, -2, -1, 0, 0, 0, 0, weber_t, true)
NEW_UNIT_LITERAL(MagneticFlux, Wb, webers, weber_t, 1, 1)

NEW_UNIT(MagneticFluxDensity, dim_magnetic_flux_density, 1, 0, -2, -1, 0, 0, 0, 0, tesla_t, true)
NEW_UNIT_LITERAL(MagneticFluxDensity, T, teslas, tesla_t, 1, 1)

NEW_UNIT(Momentum, dim_momentum, 1, 1, -1, 0, 0, 0, 0, 0, kilogram_meter_per_second_t, true)
NEW_UNIT_LITERAL(Momentum, kgmps, kilogram_meters_per_second, kilogram_meter_per_second_t, 1, 1)
NEW_UNIT_LITERAL(Momentum, Ns, newton_seconds, newton_second_t, 1, 1)

NEW_UNIT(Force, dim_force, 1, 1, -2, 0, 0, 0, 0, 0, newton_t, true)
NEW_UNIT_LITERAL(Force, newton, newtons, newton_t, 1, 1)

NEW_UNIT(Pressure, dim_pressure, 1, -1, -2, 0, 0, 0, 0, 0, pascal_t, true)
NEW_UNIT_LITERAL(Pressure, Pa, pascals, pascal_t, 1, 1)
NEW_UNIT_LITERAL(Pressure, psi, pounds_per_square_inch, psi_t, 6894757293168361.0, 1000000000000.0)
NEW_UNIT_LITERAL(Pressure, bar, bars, bar_t, 100000, 1)
NEW_UNIT_LITERAL(Pressure, atm, atmospheres, atmosphere_t, 101325, 1)
NEW_UNIT_LITERAL(Pressure, torr, torr_units, torr_t, 101325, 760)

NEW_UNIT(Stiffness, dim_stiffness, 1, 0, -2, 0, 0, 0, 0, 0, newton_per_meter_t, true)
NEW_UNIT_LITERAL(Stiffness, Npm, newtons_per_meter, newton_per_meter_t, 1, 1)
NEW_UNIT_LITERAL(Stiffness, Npmm, newtons_per_millimeter, newton_per_millimeter_t, 1000, 1)

NEW_UNIT(Compliance, dim_compliance, -1, 0, 2, 0, 0, 0, 0, 0, meter_per_newton_t, true)
NEW_UNIT_LITERAL(Compliance, mpN, meters_per_newton, meter_per_newton_t, 1, 1)
NEW_UNIT_LITERAL(Compliance, mmpN, millimeters_per_newton, millimeter_per_newton_t, 1, 1000)

NEW_UNIT(MassDensity, dim_mass_density, 1, -3, 0, 0, 0, 0, 0, 0, kilogram_per_cubic_meter_t, true)
NEW_UNIT_LITERAL(MassDensity, kgpm3, kilograms_per_cubic_meter, kilogram_per_cubic_meter_t, 1, 1)
NEW_UNIT_LITERAL(MassDensity, gpcm3, grams_per_cubic_centimeter, gram_per_cubic_centimeter_t, 1000, 1)

NEW_UNIT(AreaDensity, dim_area_density, 1, -2, 0, 0, 0, 0, 0, 0, kilogram_per_square_meter_t, true)
NEW_UNIT_LITERAL(AreaDensity, kgpm2, kilograms_per_square_meter, kilogram_per_square_meter_t, 1, 1)
NEW_UNIT_LITERAL(AreaDensity, gpm2, grams_per_square_meter, gram_per_square_meter_t, 1, 1000)

NEW_UNIT(VolumetricFlow, dim_volumetric_flow, 0, 3, -1, 0, 0, 0, 0, 0, cubic_meter_per_second_t, true)
NEW_UNIT_LITERAL(VolumetricFlow, m3ps, cubic_meters_per_second, cubic_meter_per_second_t, 1, 1)
NEW_UNIT_LITERAL(VolumetricFlow, Lps, liters_per_second, liter_per_second_t, 1, 1000)
NEW_UNIT_LITERAL(VolumetricFlow, gpm, gallons_per_minute, us_gallon_per_minute_t, 788627455, 12500000000000)

NEW_UNIT(MassFlow, dim_mass_flow, 1, 0, -1, 0, 0, 0, 0, 0, kilogram_per_second_t, true)
NEW_UNIT_LITERAL(MassFlow, kgps, kilograms_per_second, kilogram_per_second_t, 1, 1)
NEW_UNIT_LITERAL(MassFlow, gps, grams_per_second, gram_per_second_t, 1, 1000)

NEW_UNIT(Inertia, dim_inertia, 1, 2, 0, 0, 0, 0, 0, 0, kilogram_square_meter_t, true)
NEW_UNIT_LITERAL(Inertia, kgm2, kilogram_square_meters, kilogram_square_meter_t, 1, 1)
NEW_UNIT_LITERAL(Inertia, kgcm2, kilogram_square_centimeters, kilogram_square_centimeter_t, 1, 10000)
NEW_UNIT_LITERAL(Inertia, kgin2, kilogram_square_inches, kilogram_square_inch_t, 16129, 25000000)

NEW_UNIT(AngularMomentum, dim_angular_momentum, 1, 2, -1, 0, 0, 0, 0, 1, kilogram_square_meter_per_second_t, true)
NEW_UNIT_LITERAL(AngularMomentum, kgm2ps, kilogram_square_meters_per_second, kilogram_square_meter_per_second_t, 1, 1)
NEW_UNIT_LITERAL(AngularMomentum, Nms, newton_meter_seconds, newton_meter_second_t, 1, 1)

NEW_UNIT(Energy, dim_energy, 1, 2, -2, 0, 0, 0, 0, 0, joule_t, true)
NEW_UNIT_LITERAL(Energy, J, joules, joule_t, 1, 1)

NEW_UNIT(Torque, dim_energy, 1, 2, -2, 0, 0, 0, 0, 0, newton_meter_t, false)
NEW_UNIT_LITERAL(Torque, Nm, newton_meters, newton_meter_t, 1, 1)
NEW_UNIT_LITERAL(Torque, lbft, pound_feet, pound_foot_torque_t, 13558179483314004.0, 10000000000000000.0)
NEW_UNIT_LITERAL(Torque, ozin, ounce_inches, ounce_inch_torque_t, 706155181422604.0, 100000000000000000.0)

NEW_UNIT(RotationalStiffness, dim_rotational_stiffness, 1, 2, -2, 0, 0, 0, 0, -1, newton_meter_per_radian_t, true)
NEW_UNIT_LITERAL(RotationalStiffness, Nmprad, newton_meters_per_radian, newton_meter_per_radian_t, 1, 1)
NEW_UNIT_LITERAL(RotationalStiffness, Nmpdeg, newton_meters_per_degree, newton_meter_per_degree_t, 5729577951308232.0,
                 100000000000000.0)

NEW_UNIT(RotationalCompliance, dim_rotational_compliance, -1, -2, 2, 0, 0, 0, 0, 1, radian_per_newton_meter_t, true)
NEW_UNIT_LITERAL(RotationalCompliance, radpNm, radians_per_newton_meter, radian_per_newton_meter_t, 1, 1)
NEW_UNIT_LITERAL(RotationalCompliance, degpNm, degrees_per_newton_meter, degree_per_newton_meter_t, 17453292519943295.0,
                 1000000000000000000.0)

NEW_UNIT(RotationalDamping, dim_rotational_damping, 1, 2, -1, 0, 0, 0, 0, -1, newton_meter_second_per_radian_t, true)
NEW_UNIT_LITERAL(RotationalDamping, Nmsprad, newton_meter_seconds_per_radian, newton_meter_second_per_radian_t, 1, 1)
NEW_UNIT_LITERAL(RotationalDamping, Nmspdeg, newton_meter_seconds_per_degree, newton_meter_second_per_degree_t,
                 5729577951308232.0, 100000000000000.0)

NEW_UNIT(Power, dim_power, 1, 2, -3, 0, 0, 0, 0, 0, watt_t, true)
NEW_UNIT_LITERAL(Power, W, watts, watt_t, 1, 1)

NEW_UNIT(SolidAngle, dim_solid_angle, 0, 0, 0, 0, 0, 0, 0, 2, steradian_t, true)
NEW_UNIT_LITERAL(SolidAngle, sr, steradians, steradian_t, 1, 1)

NEW_UNIT(LuminousFlux, dim_luminous_flux, 0, 0, 0, 0, 0, 0, 1, 2, lumen_t, true)
NEW_UNIT_LITERAL(LuminousFlux, lm, lumens, lumen_t, 1, 1)

NEW_UNIT(Illuminance, dim_illuminance, 0, -2, 0, 0, 0, 0, 1, 2, lux_t, true)
NEW_UNIT_LITERAL(Illuminance, lx, lux, lux_t, 1, 1)

NEW_UNIT(LinearVelocityFeedforward, dim_linear_velocity_feedforward, 1, 1, -2, -1, 0, 0, 0, 0,
         volt_per_meter_per_second_t, false)
NEW_UNIT_LITERAL(LinearVelocityFeedforward, VpMps, volts_per_meter_per_second, volt_per_meter_per_second_t, 1, 1)
NEW_UNIT_LITERAL(LinearVelocityFeedforward, VpInps, volts_per_inch_per_second, volt_per_inch_per_second_t, 5000, 127)

NEW_UNIT(LinearProportionalGain, dim_linear_proportional_gain, 1, 1, -3, -1, 0, 0, 0, 0, volt_per_meter_t, true)
NEW_UNIT_LITERAL(LinearProportionalGain, VpM, volts_per_meter, volt_per_meter_t, 1, 1)
NEW_UNIT_LITERAL(LinearProportionalGain, VpIn, volts_per_inch, volt_per_inch_t, 5000, 127)

NEW_UNIT(LinearIntegralGain, dim_linear_integral_gain, 1, 1, -4, -1, 0, 0, 0, 0, volt_per_meter_second_t, true)
NEW_UNIT_LITERAL(LinearIntegralGain, VpMS, volts_per_meter_second, volt_per_meter_second_t, 1, 1)
NEW_UNIT_LITERAL(LinearIntegralGain, VpInS, volts_per_inch_second, volt_per_inch_second_t, 5000, 127)

NEW_UNIT(LinearAccelerationFeedforward, dim_linear_acceleration_feedforward, 1, 1, -1, -1, 0, 0, 0, 0,
         volt_per_meter_per_second_squared_t, false)
NEW_UNIT_LITERAL(LinearAccelerationFeedforward, VpMps2, volts_per_meter_per_second_squared,
                 volt_per_meter_per_second_squared_t, 1, 1)
NEW_UNIT_LITERAL(LinearAccelerationFeedforward, VpInps2, volts_per_inch_per_second_squared,
                 volt_per_inch_per_second_squared_t, 5000, 127)

NEW_UNIT(LinearDerivativeGain, dim_linear_derivative_gain, 1, 1, -2, -1, 0, 0, 0, 0, volt_second_per_meter_t, true)
NEW_UNIT_LITERAL(LinearDerivativeGain, VspM, volt_seconds_per_meter, volt_second_per_meter_t, 1, 1)
NEW_UNIT_LITERAL(LinearDerivativeGain, VspIn, volt_seconds_per_inch, volt_second_per_inch_t, 5000, 127)

NEW_UNIT(LinearVelocityProportionalGain, dim_linear_derivative_gain, 1, 1, -2, -1, 0, 0, 0, 0, volt_second_per_meter_t,
         false)
NEW_ALIAS_UNIT_LITERAL(LinearVelocityProportionalGain, VspM, volt_seconds_per_meter, volt_second_per_meter_t, 1, 1)
NEW_ALIAS_UNIT_LITERAL(LinearVelocityProportionalGain, VspIn, volt_seconds_per_inch, volt_second_per_inch_t, 5000, 127)

NEW_UNIT(LinearVelocityDerivativeGain, dim_linear_second_derivative_gain, 1, 1, -1, -1, 0, 0, 0, 0,
         volt_second_squared_per_meter_t, true)
NEW_UNIT_LITERAL(LinearVelocityDerivativeGain, Vs2pM, volt_seconds_squared_per_meter, volt_second_squared_per_meter_t,
                 1, 1)
NEW_UNIT_LITERAL(LinearVelocityDerivativeGain, Vs2pIn, volt_seconds_squared_per_inch, volt_second_squared_per_inch_t,
                 5000, 127)

NEW_UNIT(LinearVelocityIntegralGain, dim_linear_proportional_gain, 1, 1, -3, -1, 0, 0, 0, 0, volt_per_meter_t, false)
NEW_ALIAS_UNIT_LITERAL(LinearVelocityIntegralGain, VpM, volts_per_meter, volt_per_meter_t, 1, 1)
NEW_ALIAS_UNIT_LITERAL(LinearVelocityIntegralGain, VpIn, volts_per_inch, volt_per_inch_t, 5000, 127)

NEW_UNIT(AngularVelocityFeedforward, dim_angular_velocity_feedforward, 1, 2, -2, -1, 0, 0, 0, -1,
         volt_per_radian_per_second_t, false)
NEW_UNIT_LITERAL(AngularVelocityFeedforward, VpRadPs, volts_per_radian_per_second, volt_per_radian_per_second_t, 1, 1)
NEW_UNIT_LITERAL(AngularVelocityFeedforward, VpDegPs, volts_per_degree_per_second, volt_per_degree_per_second_t,
                 5729577951308232.0, 100000000000000.0)

NEW_UNIT(AngularProportionalGain, dim_angular_proportional_gain, 1, 2, -3, -1, 0, 0, 0, -1, volt_per_radian_t, true)
NEW_UNIT_LITERAL(AngularProportionalGain, VpRad, volts_per_radian, volt_per_radian_t, 1, 1)
NEW_UNIT_LITERAL(AngularProportionalGain, VpDeg, volts_per_degree, volt_per_degree_t, 5729577951308232.0,
                 100000000000000.0)

NEW_UNIT(AngularIntegralGain, dim_angular_integral_gain, 1, 2, -4, -1, 0, 0, 0, -1, volt_per_radian_second_t, true)
NEW_UNIT_LITERAL(AngularIntegralGain, VpRadS, volts_per_radian_second, volt_per_radian_second_t, 1, 1)
NEW_UNIT_LITERAL(AngularIntegralGain, VpDegS, volts_per_degree_second, volt_per_degree_second_t, 5729577951308232.0,
                 100000000000000.0)

NEW_UNIT(AngularAccelerationFeedforward, dim_angular_acceleration_feedforward, 1, 2, -1, -1, 0, 0, 0, -1,
         volt_per_radian_per_second_squared_t, false)
NEW_UNIT_LITERAL(AngularAccelerationFeedforward, VpRadPs2, volts_per_radian_per_second_squared,
                 volt_per_radian_per_second_squared_t, 1, 1)
NEW_UNIT_LITERAL(AngularAccelerationFeedforward, VpDegPs2, volts_per_degree_per_second_squared,
                 volt_per_degree_per_second_squared_t, 5729577951308232.0, 100000000000000.0)

NEW_UNIT(AngularDerivativeGain, dim_angular_derivative_gain, 1, 2, -2, -1, 0, 0, 0, -1, volt_second_per_radian_t, true)
NEW_UNIT_LITERAL(AngularDerivativeGain, VspRad, volt_seconds_per_radian, volt_second_per_radian_t, 1, 1)
NEW_UNIT_LITERAL(AngularDerivativeGain, VspDeg, volt_seconds_per_degree, volt_second_per_degree_t, 5729577951308232.0,
                 100000000000000.0)

NEW_UNIT(AngularVelocityProportionalGain, dim_angular_derivative_gain, 1, 2, -2, -1, 0, 0, 0, -1,
         volt_second_per_radian_t, false)
NEW_ALIAS_UNIT_LITERAL(AngularVelocityProportionalGain, VspRad, volt_seconds_per_radian, volt_second_per_radian_t, 1, 1)
NEW_ALIAS_UNIT_LITERAL(AngularVelocityProportionalGain, VspDeg, volt_seconds_per_degree, volt_second_per_degree_t,
                       5729577951308232.0, 100000000000000.0)

NEW_UNIT(AngularVelocityDerivativeGain, dim_angular_second_derivative_gain, 1, 2, -1, -1, 0, 0, 0, -1,
         volt_second_squared_per_radian_t, true)
NEW_UNIT_LITERAL(AngularVelocityDerivativeGain, Vs2pRad, volt_seconds_squared_per_radian,
                 volt_second_squared_per_radian_t, 1, 1)
NEW_UNIT_LITERAL(AngularVelocityDerivativeGain, Vs2pDeg, volt_seconds_squared_per_degree,
                 volt_second_squared_per_degree_t, 5729577951308232.0, 100000000000000.0)

NEW_UNIT(AngularVelocityIntegralGain, dim_angular_proportional_gain, 1, 2, -3, -1, 0, 0, 0, -1, volt_per_radian_t,
         false)
NEW_ALIAS_UNIT_LITERAL(AngularVelocityIntegralGain, VpRad, volts_per_radian, volt_per_radian_t, 1, 1)
NEW_ALIAS_UNIT_LITERAL(AngularVelocityIntegralGain, VpDeg, volts_per_degree, volt_per_degree_t, 5729577951308232.0,
                       100000000000000.0)

typedef Momentum Impulse;

#undef NEW_UNIT
#undef NEW_UNIT_LITERAL
#undef NEW_ALIAS_UNIT_LITERAL
