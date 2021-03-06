/** @file main.cpp
 *  @brief Orientation output in Signal K format via SensESP.
 * This file provides examples for using the Orientation library together
 * with SensESP to report vessel orientation data to a Signal K server.
 * Intended hardware is an ESP32 or ESP8266 platform and an FXOS8700/FXAS21002
 * combination accelerometer/magnetometer/gyroscope.
 */

#include <Arduino.h>
#include <Wire.h>

#include <sstream>
#include <string>

#include "sensesp_app.h"
#include "orientation_sensor.h"
#include "signalk_output.h"
#include "sensors/bme280.h"
#include "transforms/linear.h"
#include "transforms/angle_correction.h"
#include "sensesp_app_builder.h"
// If using the Temperature report, then include linear.h as the
// linear transform enables temperature readings to be calibrated.
//#include "transforms/linear.h"

// Sensor hardware details: I2C addresses and pins       
#define BOARD_ACCEL_MAG_I2C_ADDR    (0x1F) ///< I2C address on Adafruit breakout board
#define BOARD_GYRO_I2C_ADDR         (0x21) ///< I2C address on Adafruit breakout board
#if defined( ESP8266 )
  #define PIN_I2C_SDA (12)  //Adjust to your board. A value of -1
  #define PIN_I2C_SCL (14)  // will use default Arduino pins.
#elif defined( ESP32 )
  #define PIN_I2C_SDA (21)  //Adjust to your board. A value of -1
  #define PIN_I2C_SCL (22)  // will use default Arduino pins.
#endif

// How often orientation parameters are published via Signal K message
#define ORIENTATION_REPORTING_INTERVAL_MS (100)

// SensESP builds upon the ReactESP framework. Every ReactESP application
// defines an "app" object vs defining a "main()" method.
ReactESP app([]() {

// Some initialization boilerplate when in debug mode...
#ifndef SERIAL_DEBUG_DISABLED
  SetupSerialDebug(115200);
#endif

  /**
   * Create the global SensESPApp() object.
   * By passing the WiFi setup details in the constructor, rather than
   * relying on entering it in the device's web interface, we save about
   * 2496 bytes of heap memory (RAM). Another alternative is to use the
   * Builder pattern (sensesp_app_builder.h), but that saves only 1880 bytes.
   */
  SensESPAppBuilder builder;

  // Set whatever options you want, then create the global SensESPApp() object with get_app():
  sensesp_app = builder.set_standard_sensors(UPTIME)
              ->set_hostname("CompassBME280")
              ->set_sk_server("10.10.10.1", 3000)
              ->get_app();
   
  /**
   * The "SignalK path" identifies this sensor to the Signal K server. Leaving
   * this blank would indicate this particular sensor or transform does not
   * broadcast Signal K data.
   * If you have multiple sensors connected to your microcontroller (ESP),
   * each of them will probably have its own Signal K path variable. For
   * example, if you have two propulsion engines, and you want the RPM of
   * each of them to go to Signal K, you might have
   * sk_path_portEngine = "propulsion.port.revolutions" and
   * sk_path_starboardEngine = "propulsion.starboard.revolutions"
   * To find valid Signal K Paths look at this link (or later version):
   * @see https://signalk.org/specification/1.5.0/doc/vesselsBranch.html
   *
   * Vessel heading can be reported as headingCompass (uncorrected for
   * Deviation), headingMagnetic (corrected for Devations),
   * or as part of an attitude data group (i.e. yaw, pitch, roll).
   * All three paths are defined in the Signal K spec and have default
   * display widgets in the Signal K Instrument Panel.
   */
  const char* kSKPathHeadingCompass  = "navigation.headingCompass";
  const char* kSKPathHeadingMagnetic = "navigation.headingMagnetic";
  const char* kSKPathAttitude        = "navigation.attitude";
  /**
   * This example reports heading, pitch, and roll. If you want other parameters
   * as well, uncomment the appropriate SKpath(s) from the following.
   * Signal K v1.5 does not describe paths for roll rate and pitch rate
   * so these are provided using the same pattern as for rateOfTurn.
   * Signal K v1.5 says path for temperature can include zone.
   * Replace ecompass with a different zone if desired.
   * Signal K v1.5 does not describe a path for acceleration.
   */
  //const char* kSKPathTurnRate    = "navigation.rateOfTurn";
  //const char* kSKPathRollRate    = "navigation.rateOfRoll";
  //const char* kSKPathPitchRate   = "navigation.rateOfPitch";
  //const char* kSKPathTemperature =
  //               "environment.inside.ecompass.temperature";
  //const char* kSKPathAccel       = "sensors.accelerometer.accel_xyz";
  /**
   * The following SKpaths are useful when performing magnetic calibration,
   * and for confirming that the current magnetic environment of the sensor
   * is unchanged from the most recent saved calibration. None of these
   * parameters has a defined path in Signal K, so they may be changed to suit.
   * 
   * For more details and suggestions on how to perform magnetic calibration,
   * see the Wiki at
   * @see https://github.com/BjarneBitscrambler/SignalK-Orientation/wiki
   */
  const char* kSKPathMagFit          = "orientation.calibration.magfit";
  const char* kSKPathMagFitTrial     = "orientation.calibration.magfittrial";
  const char* kSKPathMagSolver       = "orientation.calibration.magsolver";
  //const char* kSKPathMagInclination  = "orientation.calibration.maginclination";
  //const char* kSKPathkMagBValue      = "orientation.calibration.magmagnitude";
  //const char* kSKPathkMagBValueTrial = "orientation.calibration.magmagnitudetrial";
  //const char* kSKPathkMagNoise       = "orientation.calibration.magnoise";
  //const char* kSKPathkMagCalValues   = "orientation.calibration.magvalues";

  /**
   * If you are creating a new Signal K path that does not
   * already exist in the specification, it is best to
   * define "metadata" that describes your new value. This
   * metadata will be reported to the Signal K server the first
   * time your sensor reports its value(s) to the server.
   */
  // Uncomment from the following example metadata as needed, or create
  // your own as needed.
  //   SKMetadata* metadata_accel = new SKMetadata();
  //   metadata_accel->description_ = "Acceleration in X,Y,Z axes";
  //   metadata_accel->display_name_ = "Accelerometer";
  //   metadata_accel->short_name_ = "Accel";
  //   metadata_accel->units_ = "m/s^2";
  //
  //   SKMetadata* metadata_rate_of_roll = new SKMetadata();
  //   metadata_rate_of_roll->description_ =
  //        "Rate of Roll about bow-stern axis";
  //   metadata_rate_of_roll->display_name_ = "Roll Rate";
  //   metadata_rate_of_roll->short_name_ = "Roll Rate";
  //   metadata_rate_of_roll->units_ = "rad/s";
  //
  //   SKMetadata* metadata_rate_of_pitch = new SKMetadata();
  //   metadata_rate_of_pitch->description_ =
  //        "Rate of Pitch about port-starboard axis";
  //   metadata_rate_of_pitch->display_name_ = "Pitch Rate";
  //   metadata_rate_of_pitch->short_name_ = "Pitch Rate";
  //   metadata_rate_of_pitch->units_ = "rad/s";
  //
  //   SKMetadata* metadata_temperature = new SKMetadata();
  //   metadata_temperature->description_ =
  //        "Temperature reported by orientation sensor";
  //   metadata_temperature->display_name_ = "Temperature at eCompass";
  //   metadata_temperature->short_name_ = "Temp";
  //   metadata_temperature->units_ = "K";

  /**
   * The "Configuration path" is combined with "/config" to formulate a URL
   * used by the RESTful API for retrieving or setting configuration data.
   * It is ALSO used to specify a path to the file system
   * where configuration data is saved on the MCU board. It should
   * ALWAYS start with a forward slash if specified. If left blank,
   * that indicates this sensor or transform does not have any
   * configuration to save, or that you're not interested in doing
   * run-time configuration.
   * These two are necessary until a method is created to synthesize them.
   * 
   * Note the hardware sensor itself has no run-time configurable items.
   * Note the empty "" for configuring the SK paths for attitude and 
   * heading: this is because the paths for these parameters are
   * prescribed by the SK spec, and default instruments expect these
   * paths. You can override them, but will then need to define your
   * own instruments to display the data.
   * 
   * Below arrangement of config paths yields this web interface structure:
   * 
       sensors->attitude
                       ->value_settings (adjusts report interval, saves mag cal)
              ->heading
                       ->value_settings (adjusts report interval, saves mag cal)
                       ->deviation      (adjusts compass deviation)
   * 
   */
  const char* kConfigPathAttitude_SK = "";
  const char* kConfigPathAttitude    = "/sensors/attitude/value_settings";
  const char* kConfigPathHeading_SKC = "";
  const char* kConfigPathHeading_SKM = "";
  const char* kConfigPathHeading     = "/sensors/heading/value_settings";
  const char* kConfigPathHeadingDev  = "/sensors/heading/deviation";
  // This example shows attitude and compass heading. If you want other parameters
  // as well, uncomment and modify the appropriate path(s) from the following 
  // or create new paths as needed.
  //   const char* kConfigPathTurnRate_SK    = "/sensors/rateOfTurn/sk";
  //   const char* kConfigPathTurnRate       = "/sensors/rateOfTurn/value_settings";
  //   const char* kConfigPathAccelXYZ       = "/sensors/acceleration/value_settings";
  //   const char* kConfigPathAccelXYZ_SK    = "/sensors/acceleration/sk";
  //   const char* kConfigPathTemperature    = "/sensors/temperature/value_settings";
  //   const char* kConfigPathTemperature_SK = "/sensors/temperature/sk";

  /**
   * Create and initialize the Orientation data source.
   * This uses a 9 Degrees-of-freedom combination sensor that provides multiple
   * orientation parameters. Selection of which particular parameters are
   * output is performed later when the value producers are created.
   * 
   * Magnetic Calibration occurs during regular runtime. After power-on, move
   * the sensor through a series of rolls, pitches and yaws. After enough
   * readings have been collected (takes 15-30 seconds when rotating the sensor
   * by hand) then the sensor should be calibrated.
   * A Magnetic Calibration can be saved in non-volatile memory so it will be
   * loaded at the next power-up. To save a calibration, use the
   * value_settings->Save_Mag_Cal entry in the sensor web interface.
   * A calibration will be valid until the sensor's magnetic environment
   * changes.
   */
  auto* orientation_sensor = new OrientationSensor(
      PIN_I2C_SDA, PIN_I2C_SCL, BOARD_ACCEL_MAG_I2C_ADDR, BOARD_GYRO_I2C_ADDR);

  /*
   * Create the desired outputs from the orientation sensor. Note that the physical
   * sensor is read at whatever rate is specified in the Sensor Fusion library's
   * build.h file (#define FUSION_HZ), currently set to 40 Hz. Fusion
   * calculations are run at that same rate. This is different than, and
   * usually faster than, the rate at which orientation parameters are output.
   * Reportng orientation values within SensESP can happen at any desired
   * rate, though if it is more often than the fusion rate then
   * there will be duplicated values. This example uses a 10 Hz output rate.
   * It is not necessary that all the values be output at the same rate (for
   * example, it likely makes sense to report temperature at a slower rate).
   */
  
  // Create the Compass Heading and Magnetic Heading outputs. The difference
  // between the two, in this example, is that the Magnetic Heading passes
  // through a transform allowing one to correct for a fixed offset (such
  // as occurs when the sensor's axis is not perfectly parallel with the 
  // vessel's stern-bow axis).
  auto* sensor_heading = new OrientationValues(
      orientation_sensor, OrientationValues::kCompassHeading,
      ORIENTATION_REPORTING_INTERVAL_MS, kConfigPathHeading);
  sensor_heading->connect_to(
      new SKOutputNumber(kSKPathHeadingCompass, kConfigPathHeading_SKC))
      //pass through transform. Set initial offset to 0.0 radians.
      ->connect_to( new AngleCorrection( 0.0, 0.0, kConfigPathHeadingDev) )
      //an optional, more complex transform is a CurveInterpolator
      //->connect_to( new CurveInterpolator( NULL, "/sensors/heading/Magnetic") )
      ->connect_to(
      new SKOutputNumber(kSKPathHeadingMagnetic, kConfigPathHeading_SKM));
  
  // Create the Attitude output (yaw, pitch, roll). Note that this
  // output does not pass through any transform to correct for residual
  // deviation due to e.g. mounting offsets.
  auto* sensor_attitude = new AttitudeValues(
      orientation_sensor, ORIENTATION_REPORTING_INTERVAL_MS,
      kConfigPathAttitude);
  sensor_attitude->connect_to(
      new SKOutputAttitude(kSKPathAttitude, kConfigPathAttitude_SK));

  /**
   * The following outputs are useful when calibrating. See the wiki at
   * @see https://github.com/BjarneBitscrambler/SignalK-Orientation/wiki
   * for details on how to interpret the values. None are recognized
   * in the Signal K spec, so there is no prescribed SK path that they
   * need to be sent to. 
   * 
   * Because there are quite a few parameters, and they are likely only
   * referred to infrequently (i.e. when calibrating, or when magnetic
   * disturbances are suspected), you may want to configure the Signal K
   * instrument panel to display these paths on a secondary screen, 
   * separate from the primary navigation screen.
   */
  auto* sensor_cal_fit = new OrientationValues(
      orientation_sensor, OrientationValues::kMagCalFitInUse,
      ORIENTATION_REPORTING_INTERVAL_MS * 10, "");
  sensor_cal_fit->connect_to(
      new SKOutputNumber(kSKPathMagFit, ""));

  auto* sensor_cal_candidate = new OrientationValues(
      orientation_sensor, OrientationValues::kMagCalFitTrial,
      ORIENTATION_REPORTING_INTERVAL_MS * 10, "");
  sensor_cal_candidate->connect_to(
      new SKOutputNumber(kSKPathMagFitTrial, ""));

  auto* sensor_cal_order = new OrientationValues(
      orientation_sensor, OrientationValues::kMagCalAlgorithmSolver,
      ORIENTATION_REPORTING_INTERVAL_MS * 10, "");
  sensor_cal_order->connect_to(
      new SKOutputNumber(kSKPathMagSolver, ""));

  // auto* sensor_b_mag = new OrientationValues(
  //     orientation_sensor, OrientationValues::kMagInclination,
  //     ORIENTATION_REPORTING_INTERVAL_MS * 10, "");
  // sensor_b_mag->connect_to(
  //     new SKOutputNumber(kSKPathMagInclination, ""));

  // auto* sensor_b_mag = new OrientationValues(
  //     orientation_sensor, OrientationValues::kMagFieldMagnitude,
  //     ORIENTATION_REPORTING_INTERVAL_MS * 10, "");
  // sensor_b_mag->connect_to(
  //     new SKOutputNumber(kSKPathkMagBValue, ""));

  // auto* sensor_b_mag = new OrientationValues(
  //     orientation_sensor, OrientationValues::kMagFieldMagnitudeTrial,
  //     ORIENTATION_REPORTING_INTERVAL_MS * 10, "");
  // sensor_b_mag->connect_to(
  //     new SKOutputNumber(kSKPathkMagBValueTrial, ""));

  // auto* sensor_b_mag = new OrientationValues(
  //     orientation_sensor, OrientationValues::kMagNoiseCovariance,
  //     ORIENTATION_REPORTING_INTERVAL_MS * 10, "");
  // sensor_b_mag->connect_to(
  //     new SKOutputNumber(kSKPathkMagNoise, ""));

  // This report is a consolidation of all the above magnetic cal
  // values and will need a custom instrument to display.
  // auto* sensor_mag_cal = new MagCalValues(
  //     orientation_sensor, ORIENTATION_REPORTING_INTERVAL_MS * 10, "");
  // sensor_mag_cal->connect_to(
  //     new SKOutputMagCal(kSKPathkMagCalValues, ""));

  // Create a BME280, which represents the physical sensor.
  // 0x77 is the default address. Some chips use 0x76, which is shown here.
  auto* bme280 = new BME280(0x76);

  // If you want to change any of the settings that are set by
  // Adafruit_BME280::setSampling(), do that here, like this:
  // bme280->adafruit_bme280->setSampling(); // pass in the parameters you want

  // Define the read_delays you're going to use:
  const uint read_delay = 1000;            // once per second
  const uint pressure_read_delay = 60000;  // once per minute

  // Create a BME280Value, which is used to read a specific value from the
  // BME280, and send its output to Signal K as a number (float). This one is for
  // the temperature reading.
  auto* bme_temperature =
      new BME280Value(bme280, BME280Value::temperature, read_delay, "/Outside/Temperature");

  bme_temperature->connect_to(
      new SKOutputNumber("environment.outside.temperature"));

  // Do the same for the barometric pressure value. Its read_delay is longer,
  // since barometric pressure can't change all that quickly. It could be much
  // longer for that reason.
  auto* bme_pressure = new BME280Value(bme280, BME280Value::pressure, pressure_read_delay,
                                       "/Outside/Pressure");

  bme_pressure->connect_to(new SKOutputNumber("environment.outside.pressure"));

  // Do the same for the humidity value.
  auto* bme_humidity =
      new BME280Value(bme280, BME280Value::humidity, read_delay, "/Outside/Humidity");

  bme_humidity->connect_to(new SKOutputNumber("environment.outside.humidity"));

  // Start the SensESP application running
  sensesp_app->enable();
});
