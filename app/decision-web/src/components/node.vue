<template>
  <div class="node">
  </div>
</template>

<script setup lang="ts" name="Node">
  import { ref } from 'vue';
  const node = ref<HTMLDivElement | null>(null);
</script>

<style scoped>
</style>