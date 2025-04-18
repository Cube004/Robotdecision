
<template>
  <div>
    <canvas ref="backgroundLayer"></canvas>
    <!-- <canvas ref="operationLayer"></canvas> -->
  </div>
</template>

<script setup lang="ts" name="App">
  import { onMounted, ref } from 'vue';
  const backgroundLayer = ref<HTMLCanvasElement | null>(null);
  onMounted(() => {
    const ctx = backgroundLayer.value?.getContext('2d');
    if (backgroundLayer.value && ctx){
      backgroundLayer.value.width = window.innerWidth;
      backgroundLayer.value.height = window.innerHeight;

      ctx.fillStyle = '#F2F4F7';
      ctx.fillRect(0, 0, window.innerWidth, window.innerHeight);
      const gridInterval = window.innerWidth / 100;
      // 计算最大有效索引
      const maxXIndex = Math.floor(backgroundLayer.value.width / gridInterval);
      const maxYIndex = Math.floor(backgroundLayer.value.height / gridInterval);

      for (let xIndex = 0; xIndex <= maxXIndex; xIndex++) {
          for (let yIndex = 0; yIndex <= maxYIndex; yIndex++) {
              const x = xIndex * gridInterval;
              const y = yIndex * gridInterval;

              ctx.beginPath();
              ctx.fillStyle = '#E2E4EC';
              ctx.arc(x, y, 1.5 , 0, Math.PI * 2);
              ctx.fill();
          }
      }
    }
  })
</script>


<style scoped>
  canvas {
    position: absolute;
    top: 0;
    left: 0;
    width: 100%;
    height: 100%;
  }
</style>
