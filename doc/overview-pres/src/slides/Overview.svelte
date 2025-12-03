<script>
  import Slide from '../lib/Slide.svelte';
  import Admonishment from '../lib/Admonishment.svelte';
</script>

<section data-background="assets/BG1.png" data-background-opacity="0.3">
  <h1 class="font-sans font-thin text-8xl">What is Benji?</h1>
</section>

<Slide title="The Goal">
  <div class="text-3xl space-y-8">
    <div class="fragment">
      <Admonishment type="important" title="Inspiration">
        I was out of town during the Marshall Fire a few years back. That was not a good day.
      </Admonishment>
    </div>
    <div class="fragment">
      <p class=""><strong>I want to know how my apartment is doing when I'm traveling.</strong></p>
      <p class="italic font-thin">This means the robot must:</p>
      <ul class="list-disc list-inside space-y-4">
        <li>Navigate indoor spaces autonomously</li>
        <li>Stream video, audio, and telemetry</li>
        <li>Record patrols for later review</li>
        <li>Be internet accessible and controllable</li>
        <li>Handle its own state of charge</li>
      </ul>
    </div>
  </div>
</Slide>

<Slide title="Why?">
  <div class="text-3xl space-y-6">
    <p class="fragment text-4xl font-thin italic">Because I can. And before I did, I couldn't. Learning's fun.</p>
    <div class="fragment">
      <div>
        <h3 class="text-4xl font-semibold text-blue-400">Practical Experience</h3>
        <p class="text-2xl mt-2">Good practice with embedded systems, RTOS, computer vision, and control theory</p>
        <p class="text-2xl mt-2 font-thin">(And it's not too different from a spacecraft, if you squint a bit)</p>
      </div>
      <div>
        <h3 class="text-4xl font-semibold text-green-400">Context for Trying New Things</h3>
        <p class="text-2xl mt-2">Proving ground for new tools, ideas, and skills I want to try</p>
      </div>
      <div>
        <h3 class="text-4xl font-semibold text-purple-400">Architecture Practice</h3>
        <p class="text-2xl mt-2">To get better at complex architecture, you have to architect complex things.</p>
      </div>
    </div>
  </div>
</Slide>
