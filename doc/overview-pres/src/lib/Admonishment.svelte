<script>
  export let type = "note"; // note, warning, tip, important, caution
  export let title = "";

  const styles = {
    note: {
      border: "border-blue-500",
      bg: "bg-blue-500/10",
      icon: "i",
      iconBg: "bg-blue-500",
      defaultTitle: "Note"
    },
    warning: {
      border: "border-yellow-500",
      bg: "bg-yellow-500/10",
      icon: "!",
      iconBg: "bg-yellow-500",
      defaultTitle: "Warning"
    },
    tip: {
      border: "border-green-500",
      bg: "bg-green-500/10",
      icon: "?",
      iconBg: "bg-green-500",
      defaultTitle: "Tip"
    },
    important: {
      border: "border-purple-500",
      bg: "bg-purple-500/10",
      icon: "!",
      iconBg: "bg-purple-500",
      defaultTitle: "Important"
    },
    caution: {
      border: "border-red-500",
      bg: "bg-red-500/10",
      icon: "!",
      iconBg: "bg-red-500",
      defaultTitle: "Caution"
    }
  };

  $: style = styles[type] || styles.note;
  $: displayTitle = title || style.defaultTitle;
</script>

<div class="rounded-lg border-l-4 {style.border} {style.bg} p-4 my-4">
  <div class="flex items-start gap-3">
    <div class="flex-shrink-0 w-6 h-6 rounded-full {style.iconBg} flex items-center justify-center text-sm font-bold text-black">
      {style.icon}
    </div>
    <div class="flex-1">
      <p class="font-semibold text-xl mb-1">{displayTitle}</p>
      <div class="text-lg opacity-90">
        <slot />
      </div>
    </div>
  </div>
</div>
