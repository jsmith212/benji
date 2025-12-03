<script>
  import Admonishment from '../lib/Admonishment.svelte';
import Slide from '../lib/Slide.svelte';
</script>

<section data-background="assets/BG3.png" data-background-opacity="0.3">
  <h1 class="font-sans font-thin text-8xl">GNC</h1>
  <h2 class="font-sans font-light text-4xl mt-4">Guidance, Navigation, and Control</h2>
</section>

<Slide title="Dynamics Model">
  <div class="grid grid-cols-2 gap-8">
    <div class="text-xl space-y-4">
      <p>5-state differential drive with motor dynamics:</p>
      <div class="text-lg mt-4">
        <p class="font-semibold text-yellow-400 mb-2">Kinematics</p>
        <div class="space-y-1">
          <p>$\dot x = \frac{'{'}v_l + v_r{'}'}{'{'}2{'}'} \cos\theta$</p>
          <p>$\dot y = \frac{'{'}v_l + v_r{'}'}{'{'}2{'}'} \sin\theta$</p>
          <p>$\dot\theta = \frac{'{'}v_r - v_l{'}'}{'{'}L{'}'}$</p>
        </div>
      </div>
      <div class="text-lg mt-4">
        <p class="font-semibold text-blue-400 mb-2">Motor Dynamics</p>
        <div class="space-y-1">
          <p>$\dot v_l = \frac{'{'}u_l - v_l{'}'}{'{'}\tau{'}'}$</p>
          <p>$\dot v_r = \frac{'{'}u_r - v_r{'}'}{'{'}\tau{'}'}$</p>
        </div>
      </div>
    </div>
    <div class="text-lg space-y-4">
      <p class="font-semibold text-green-400">Why include motor lag?</p>
      <ul class="list-disc list-inside space-y-2 text-base">
        <li>Commanded ≠ actual wheel velocity</li>
        <li>Time constant $\tau$ ~ 200ms</li>
        <li>Ignoring it → oscillation, overshoot</li>
        <li>Controller can compensate if modeled</li>
      </ul>
      <div class="mt-6 p-3 bg-gray-800 rounded text-base">
        <p class="text-gray-400">State vector:</p>
        <p class="font-mono">$\mathbf{'{'}x{'}'} = [x, y, \theta, v_l, v_r]^T$</p>
      </div>
    </div>
  </div>
</Slide>

<Slide title="Trajectory Tracking">
  <div class="text-xl space-y-3">
    <div class="mt-3 p-3 bg-gray-800/50 rounded text-base">
      <p class="font-semibold text-green-400 mb-1">Error Dynamics</p>
      <p class="text-center">$\dot{'{'}\mathbf{'{'}e{'}'}{'}'}  = A(\mathbf{'{'}x{'}'}_{'{'}ref{'}'}) \mathbf{'{'}e{'}'} + B \delta\mathbf{'{'}u{'}'}$</p>
      <p class="text-gray-400 mt-2 text-sm">
        where $\mathbf{'{'}e{'}'} = \mathbf{'{'}x{'}'} - \mathbf{'{'}x{'}'}_{'{'}ref{'}'}$ is the tracking error, and
        $A(\mathbf{'{'}x{'}'}_{'{'}ref{'}'})$ is the linearized system matrix about the reference trajectory.
      </p>
    </div>
    <p>Control decomposes into <span class="text-yellow-400">feedforward</span> + <span class="text-blue-400">feedback</span>:</p>
    <div class="text-center text-2xl my-3">
      $\mathbf{'{'}u{'}'} = \mathbf{'{'}u{'}'}_{'{'}ref{'}'} + \delta\mathbf{'{'}u{'}'}$
    </div>
    <div class="grid grid-cols-2 gap-8">
      <div>
        <p class="font-semibold text-yellow-400 mb-2">Feedforward $\mathbf{'{'}u{'}'}_{'{'}ref{'}'}$</p>
        <ul class="list-disc list-inside space-y-1 text-base">
          <li>Computed from reference trajectory</li>
          <li>Pure feedback always lags</li>
          <li>Carries the bulk of control effort</li>
        </ul>
      </div>
      <div>
        <p class="font-semibold text-blue-400 mb-2">Feedback $\delta\mathbf{'{'}u{'}'}$</p>
        <ul class="list-disc list-inside space-y-1 text-base">
          <li>Corrects for minor deviations from calculated feedforward</li>
          <li>LQR: $\delta\mathbf{'{'}u{'}'} = -K\mathbf{'{'}e{'}'}$</li>
          <li>Assume error is small, compute $K$ offline</li>
        </ul>
      </div>
    </div>
  </div>
</Slide>

<Slide title="Controller Comparison">
  <div class="grid grid-cols-2 gap-6">
    <div>
      <img src="assets/gnc_comparison.png" alt="Controller trajectory comparison" class="rounded" />
    </div>
    <div class="text-lg">
      <Admonishment type="warning" title="Rough Simulation">
        These results are from a quick Jupyter notebook that doesn't use real parameters or tuned gains. It's also run on a desktop CPU in Python. Real performance will vary.
      </Admonishment>
      <table class="w-full text-base">
        <thead>
          <tr class="text-left border-b border-gray-600">
            <th class="py-2">Controller</th>
            <th class="py-2">Time</th>
            <th class="py-2">RMS Error</th>
            <th class="py-2">Notes</th>
          </tr>
        </thead>
        <tbody>
          <tr class="border-b border-gray-700">
            <td class="py-2 text-yellow-400 font-semibold">LQR+FF</td>
            <td class="py-2">&lt;0.1ms</td>
            <td class="py-2">2.3 cm</td>
            <td class="py-2">Fixed gain, compute K offline</td>
          </tr>
          <tr class="border-b border-gray-700">
            <td class="py-2 text-red-400 font-semibold">QP-MPC</td>
            <td class="py-2">~2ms</td>
            <td class="py-2">4.0 cm</td>
            <td class="py-2">Relinearize each step, handle $|u| \le u_{'{'}max{'}'}$</td>
          </tr>
          <tr>
            <td class="py-2 text-green-400 font-semibold">NL-MPC</td>
            <td class="py-2">~20ms</td>
            <td class="py-2">2.6 cm</td>
            <td class="py-2">Full nonlinear</td>
          </tr>
        </tbody>
      </table>
    </div>
  </div>
</Slide>
