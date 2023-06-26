<!-- Variables -->
<div style="text-align: center;">
  <h1>B.ob - Bertrandt office bot</h1>
<h2>Goal for 2023-07-03:<br></h2>
<strong>Control b.ob over bluetooth with full directions control.</strong>
  </p> Remote Stop All
  </p> Change gamepad input (more precise controls)
  </p> Screwing hardware (hold b.ob upside down, nothing falls out)
  </p> Emergency Button
</p><strong>Will be tested by the PO with the user manual.</strong>

<h2>Last goal for 2023-06-19: FAILED<br></h2>
<strong>Control b.ob with the XBox to drive Forward, Backward and turn on point.</strong>
PWM on Jetson Nano is not working -> no motor movement. Start working on RaspberryPi until the Jetson will work again, start of creating a cross platform code.
Emergency button is still not implemented, due to missing hardware.

<h1>List of next ToDo's/Ideas:</h1>
Driving straight must be straight -> Implement the rotation sensor and contol the rotation. (Including documentation)</p>
Shutdown Jetson/Pi over bluetooth gamepad. (Add to gamepad documentation)</p>
Read the alarms pins from the motor drivers. (Check the pin voltage compatibility)</p>
Read the speed pins from the motor drivers. (Check the pin voltage compatibility)</p>

<h1>Definition of Done</h1>
With each new increment, b.ob needs to fulfill the following points:</p>
<strong>Safety: b.ob cannot move two cotton blocks stacked on top of each other.</strong></p>
<strong>Safety: b.obs internat components are safe</strong></p>
<strong>After adding the new feature: run a system test with a colleague</strong></p>
<strong>Merge the new code for the increment with a pull request</strong></p>
<strong>Get a code review by one of your team members (Clean Code)</strong></p>

<h1>Team Agreements</h1>
As a team we hold each other accountable to the following rules:<p>
  <h2>We will be in office three times a week, on Monday, Tuesday and Wednesday.</h2>
  <h2>Our communication channels are a shared Whatsapp Group for the Devs, and we will react to the PO/SM on a personal E-Mail Account within 24h</h2>
  <h2>We do pair working for three hours a week.</h2>
  <table>
  <tr>
  <th>Time</th>
  <th>Mo</th>
  <th>Tu</th>
  <th>We</th>
  <th>Th</th>
  <th>Fr</th>
  </tr>
  <tr>
    <td>10:00</td>
    <td></td>
    <td>Jan and Mohammed</td>
    <td>Yigit and Jan</td>
    <td></td>
    <td></td>
  </tr>
    <tr>
    <td>14:00</td>
    <td></td>
    <td>Yigit and Mohammed</td>
    <td></td>
    <td></td>
    <td></td>
  </tr>
    </table>
___


