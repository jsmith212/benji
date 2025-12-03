<script>
  import Slide from '../lib/Slide.svelte';
</script>

<Slide title="About Me">
  <div class="grid grid-cols-3 gap-8">
    <div class="col-span-1">
      <!-- Replace with your photo if desired -->
      <img class="w-48 h-48 mx-auto bg-gray-700 rounded-full flex items-center justify-center text-6xl" src="assets/me.jpg" />
    </div>
    <div class="col-span-2 text-left text-2xl space-y-4">
      <h2 class="text-4xl font-semibold">Jackson Smith</h2>
      <p class="text-xl text-gray-400">FSW Engineer</p>

      <div class="space-y-3 mt-6">
        <div>
          <span class="text-blue-400 font-semibold">Current Role:</span>
          <span>Onboard Software Architect at Loft Orbital</span>
        </div>
        <div>
          <span class="text-green-400 font-semibold">Education:</span>
          <span>Aerospace Engineering, GNC Focus</span>
        </div>
        <div>
          <span class="text-red-400 font-semibold">Experience:</span>
          <span>Safety-Critical Real-Time FSW, Microsat Payload Integration SW</span>
        </div>
        <div>
          <span class="text-purple-400 font-semibold">Spacecraft:</span>
          <span>Dream Chaser, Orbital Reef, YAM-6, YAM-7, YAM-8, YAM-9, and YAC-1</span>
        </div>
        <div>
          <span class="text-yellow-400 font-semibold">Interests:</span>
          <span>Embedded applications, operating systems, controls, infrastructure</span>
        </div>
        <div>
          <span class="text-orange-400 font-semibold">Main Languages:</span>
          <span>C/C++, Go, Python</span>
        </div>
      </div>
    </div>
  </div>
</Slide>
