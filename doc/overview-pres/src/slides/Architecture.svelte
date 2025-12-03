<script>
  import Slide from '../lib/Slide.svelte';
</script>

<section data-background="assets/BG2.png" data-background-opacity="0.3">
  <h1 class="font-sans font-thin text-8xl">Architecture</h1>
</section>

<Slide title="Physical Design">
  <div class="grid grid-cols-2 gap-8 text-xl">
    <div>
      <img src="assets/benji.jpg" alt="Benji robot" class="rounded-lg shadow-lg max-h-80 mx-auto" />
      <div>
        <h3 class="text-2xl font-semibold mb-2 text-purple-400">Integration</h3>
        <ul class="list-disc list-inside space-y-1">
          <li>Custom PCB for MCU, motor drivers, and sensors</li>
          <li>RasPi Expansion Header compatible</li>
        </ul>
      </div>
    </div>
    <div class="space-y-4">
      <div>
        <h3 class="text-2xl font-semibold mb-2 text-yellow-400">Actuators</h3>
        <ul class="list-disc list-inside space-y-1">
          <li>4x DC gearmotors, Differential Drive</li>
          <li>Servo for camera tilt</li>
        </ul>
      </div>
      <div>
        <h3 class="text-2xl font-semibold mb-2 text-blue-400">Sensors</h3>
        <ul class="list-disc list-inside space-y-1">
          <li>6-axis IMU (LS6DSMTR) + 3-axis magnetometer (LIS2MDLTR)</li>
          <li>4x quadrature wheel encoders</li>
          <li>Linear ToF distance sensor (VL35L1X)</li>
          <li>Camera (RasPi CM3)</li>
        </ul>
      </div>
      <div>
        <h3 class="text-2xl font-semibold mb-2 text-green-400">Compute</h3>
        <ul class="list-disc list-inside space-y-1">
          <li><span class="font-mono">STM32F405</span> - Real-time control</li>
          <li><span class="font-mono">Raspberry Pi 5</span> - Vision, planning, and API</li>
        </ul>
      </div>
  </div>
</Slide>

<Slide title="Software Stack">
  <div class="grid grid-cols-2 gap-6">
    <div>
      <img src="assets/benji_sw_arch.png" alt="Benji software architecture diagram" />
    </div>
    <div class="text-lg space-y-4">
      <div>
        <h3 class="text-xl font-semibold mb-2 text-blue-400">Compute - Yocto Linux</h3>
        <ul class="list-disc list-inside space-y-1">
          <li>Custom distro, via Kas (AutonomOS via Kas)</li>
          <li>Docker Support</li>
          <li>OTA Updates via Mender</li>
          <li>C++/Go Microservices over NATS</li>
        </ul>
        <p class="text-base text-gray-400">SLAM, path planning, API/UI</p>
      </div>
      <div>
        <h3 class="text-xl font-semibold mb-2 text-yellow-400">RT Core - Zephyr RTOS</h3>
        <ul class="list-disc list-inside space-y-1">
          <li>App-layer components in C++</li>
          <li>Command/Telem via Protobuf or debug shell</li>
          <li>Debug shell, DFU updates</li>
        </ul>
        <p class="text-base text-gray-400">Sensor acquisition, actuator control, short-horizon GNC</p>
      </div>
      <div class="text-base">
        <span class="text-gray-400">Link:</span>
        <span class="font-mono text-green-400">Protobuf</span> over UART with custom framing
      </div>
    </div>
  </div>
</Slide>

