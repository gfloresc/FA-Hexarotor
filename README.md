# FA-Hexarotor
Code for the fully actuated Hexa-rotor

<p>To update the original PX4 firmware from the PX4 Autopilot repository, first, you need to clone the master repository into your computer with the following command</p>

$ git clone https://github.com/PX4/PX4-Autopilot.git

<p>After that, one must replace the folders from this repository to the respective folders of the original PX4 firmware</p>
<ol>
  <li>the folder MultirotorMixer must be replaced in PX4-Autopilot/src/lib/mixer/</li>
  <li>the folder mc_att_control must be replaced in PX4-Autopilot/src/modules/</li>
  <li>the contents files in mc_pos_control must be replaced in PX4-Autopilot/src/modules/mc_pos_control/PositionControl/</li>
  <li>the folder mc_rate_control must be replaced in PX4-Autopilot/src/modules/</li>
</ol>
<p>Once all the files are replaced, it is necessary to build the firmware for the first time</p>
$ DONT_RUN=1 make px4_sitl gazebo

<p>this command will generate a new folder in</p>
<p>PX4-Autopilot/build/ named px4_sitl_default. In this folder, we will replace the file mixer_multirotor_normalized.generated.h in PX4-Autopilot/build/px4_sitl_default/src/lib/mixer/MultirotorMixer/; by replacing this file, we modify the mixer parámeters enabling the fully actuation mixing in the Hexa-rotor.</p>

<p>Finally, for simulation only, it is required to replace the last file typhoon_h480.sdf in PX4-Autopilot/Tools/sitl_gazebo/models/typhoon_h480/; this file is a modification of the Hexa-rotor model rotating the motors according to the calculated mixing matrix.</p>

<p>Now it is possible to run the SITL simulation by running</p>
$ make px4_sitl gazebo_typhoon_h480</p>

