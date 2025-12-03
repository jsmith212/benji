<script>
  import Slide from '../lib/Slide.svelte';
</script>

<section data-background="assets/BG1.png" data-background-opacity="0.3">
  <h1 class="font-sans font-thin text-8xl">Status & Roadmap</h1>
</section>

<Slide title="Current State">
  <div class="text-2xl space-y-6">
    <div class="grid grid-cols-2 gap-8 mt-6">
      <div>
        <h3 class="text-2xl font-semibold text-green-400 mb-3">Done</h3>
        <ul class="list-disc list-inside space-y-2 text-xl">
          <li>Mechanical updates</li>
          <li>Architecture</li>
          <li>First cut at RT/Compute API spec (Protobuf)</li>
          <li>GNC algorithm design</li>
          <li>Build system & tooling</li>
          <li>Operating system configuration (RT & Compute)</li>
          <li>Hardware selection</li>
          <li>Yocto base image</li>
        </ul>
      </div>
      <div>
        <h3 class="text-2xl font-semibold text-yellow-400 mb-3">In Progress</h3>
        <ul class="list-disc list-inside space-y-2 text-xl">
          <li>RT firmware</li>
          <li>PCB design</li>
          <li>RT Gateway Service</li>
          <li>Compute API Service</li>
        </ul>
        <hr class="my-6" />
        <h3 class="text-2xl font-semibold text-red-400 mb-3">Future Work</h3>
        <ul class="list-disc list-inside space-y-2 text-xl">
          <li>DFU on RT core</li>
          <li>PCB fab and board bring-up</li>
          <li>Visual SLAM</li>
          <li>Trajectory planning</li>
          <li>Robust simulation and testing environment (e.g. NASA Trick)</li>
        </ul>
      </div>
    </div>
  </div>
</Slide>
