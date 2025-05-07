<template>
  <div>
    <!-- 工具栏 -->
    <div class="tool-menu">
      <div class="tool-group">
        <!-- 选择工具 -->
        <button
          class="tool-button"
          :class="{ active: activeToolId === 'cursor' }"
          @click="selectTool('cursor')"
          title="选择工具">
          <svg t="1746187746043" viewBox="0 0 1024 1024" version="1.1" xmlns="http://www.w3.org/2000/svg" p-id="9466" width="20"><path d="M838.784 521.088L241.888 126.976 284.8 842.144l201.28-227.136 162.816 282.016 83.136-48-161.152-279.136 267.904-48.8z" fill="#FFFFFF" p-id="9467"></path><path d="M596.096 581.568l157.76 273.312-110.816 64-159.712-276.64-212.16 239.456L224 96l656.832 433.696-284.736 51.872z m-50.432-23.36l251.04-45.728L259.776 157.952l38.72 644.672 190.336-214.848 165.92 287.392 55.424-32-164.512-284.928z" fill="#5D6D7E" p-id="9468"></path></svg>
          <span class="tool-label" v-if="showLabels">选择</span>
        </button>
      </div>

      <div class="divider"></div>

      <div class="tool-group">
        <!-- 添加任务节点 -->
        <button
          class="tool-button"
          :class="{ active: activeToolId === 'renderTaskNode' }"
          @click="selectTool('renderTaskNode')"
          title="添加任务节点">
          <svg width="20" height="20" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
            <rect x="2.9" y="4.9" width="18.2" height="14.2" rx="3.1" stroke="currentColor" stroke-width="1.8"></rect>
          </svg>
          <span class="tool-label" v-if="showLabels">添加节点</span>
        </button>

        <!-- 添加连接线 -->
        <button
          class="tool-button"
          :class="{ active: activeToolId === 'renderNodeEdge' }"
          @click="selectTool('renderNodeEdge')"
          title="连接两个节点">
          <svg width="20" height="20" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
            <path d="M18.7491 7.72104L11.525 7.72104L11.525 9.62563L18.7491 9.62563L15.6132 12.7615C15.5222 12.8494 15.4497 12.9545 15.3998 13.0706C15.3499 13.1868 15.3236 13.3118 15.3225 13.4382C15.3214 13.5647 15.3455 13.6901 15.3934 13.8071C15.4413 13.9241 15.512 14.0305 15.6014 14.1199C15.6908 14.2093 15.7971 14.28 15.9141 14.3279C16.0312 14.3758 16.1566 14.3999 16.283 14.3988C16.4095 14.3977 16.5344 14.3714 16.6506 14.3215C16.7668 14.2716 16.8719 14.199 16.9597 14.1081L21.7212 9.3466C21.8997 9.16802 22 8.92585 22 8.67333C22 8.42082 21.8997 8.17864 21.7212 8.00006L16.9597 3.2386C16.8719 3.14765 16.7668 3.0751 16.6506 3.02519C16.5344 2.97528 16.4095 2.94901 16.283 2.94791C16.1566 2.94681 16.0312 2.97091 15.9141 3.01879C15.7971 3.06667 15.6908 3.13738 15.6014 3.2268C15.512 3.31621 15.4413 3.42254 15.3934 3.53957C15.3455 3.6566 15.3214 3.782 15.3225 3.90844C15.3236 4.03489 15.3499 4.15985 15.3998 4.27603C15.4497 4.39221 15.5222 4.4973 15.6132 4.58514L18.7491 7.72104Z" fill="currentColor"></path>
            <path d="M11.5229 9.62454C10.997 9.62454 10.5706 10.0509 10.5706 10.5768V18.1952C10.5706 19.773 9.29156 21.052 7.71375 21.052H2.95229C2.42636 21.052 2 20.6257 2 20.0998C2 19.5738 2.42636 19.1475 2.95229 19.1475H7.71375C8.23969 19.1475 8.66605 18.7211 8.66605 18.1952V10.5768C8.66605 8.99902 9.94511 7.71996 11.5229 7.71996V9.62454Z" fill="currentColor"></path>
          </svg>
          <span class="tool-label" v-if="showLabels">连接节点</span>
        </button>

        <!-- 任务编组 -->
        <button
          class="tool-button"
          :class="{ active: activeToolId === 'renderNodeGroup' }"
          @click="selectTool('renderNodeGroup')"
          title="任务编组">
          <svg t="1746630585972" width="18" height="18"  viewBox="0 0 1024 1024" version="1.1" xmlns="http://www.w3.org/2000/svg" p-id="4305">
            <path fill="currentColor" d="M875.76 751.32H771.98c-18.75 0-33.96-15.2-33.96-33.96s15.2-33.96 33.96-33.96h103.78c8.3 0 15.31-8.21 15.31-17.93V190.71c0-9.72-7.01-17.93-15.31-17.93H301.29c-8.3 0-15.31 8.21-15.31 17.93v115.93c0 18.75-15.2 33.96-33.96 33.96s-33.96-15.2-33.96-33.96V190.71c0-47.34 37.34-85.85 83.23-85.85h574.47c45.89 0 83.23 38.51 83.23 85.85v474.76c0 47.34-37.34 85.85-83.23 85.85z"  p-id="4306"></path>
            <path fill="currentColor" d="M722.71 919.14H148.24c-45.89 0-83.23-38.51-83.23-85.85V358.53c0-47.34 37.34-85.85 83.23-85.85h574.47c45.89 0 83.23 38.51 83.23 85.85v474.76c0 47.34-37.34 85.85-83.23 85.85zM148.24 340.6c-8.3 0-15.31 8.21-15.31 17.93v474.76c0 9.72 7.01 17.93 15.31 17.93h574.47c8.3 0 15.31-8.21 15.31-17.93V358.53c0-9.72-7.01-17.93-15.31-17.93H148.24z"  p-id="4307">
          </path>
          </svg>
          <span class="tool-label" v-if="showLabels">任务编组</span>
        </button>
      </div>

      <div class="divider"></div>
      <div class="tool-group">
        <!-- 图层管理 -->
        <button
          class="tool-button"
          :class="{ active: activeToolId === 'Layer' }"
          @click="selectTool('Layer')"
          title="图层管理">
          <svg t="1746510611309" width="18" height="18" class="icon" viewBox="0 0 1024 1024" version="1.1" xmlns="http://www.w3.org/2000/svg" p-id="9466">
            <path d="M1009.191632 793.5089c-12.075098 21.48958-29.983081 37.862594-51.984318 47.788733L611.839056 1001.855499a240.478639 240.478639 0 0 1-201.4904 0L64.468741 841.195302a111.950481 111.950481 0 0 1-51.37033-47.174746 39.499896 39.499896 0 0 1 15.247369-53.519288 39.499896 39.499896 0 0 1 53.519289 15.247369 30.494738 30.494738 0 0 0 15.247369 14.121724l346.289239 160.660197c41.955848 19.340622 92.405196 19.340622 134.872701 0l345.879914-160.660197a32.848359 32.848359 0 0 0 15.759026-14.735712 39.499896 39.499896 0 0 1 53.519289-15.145038 39.499896 39.499896 0 0 1 15.145037 53.519289h0.613988zM994.45592 512.711714a39.499896 39.499896 0 0 0-53.519288 15.247369 32.848359 32.848359 0 0 1-15.759026 14.735713l-345.879915 160.557865c-41.955848 19.442954-92.302865 19.442954-134.770369 0L98.135751 542.592464a33.257684 33.257684 0 0 1-15.247369-14.121724 39.499896 39.499896 0 0 0-53.519289-15.247369 39.499896 39.499896 0 0 0-15.247369 53.519289c12.075098 21.48958 29.983081 37.862594 51.472662 47.277077l345.879915 160.660197c31.927377 14.633381 66.106043 22.512894 100.694034 22.512894 34.690323 0 68.766658-7.367856 100.796366-22.103569l345.265926-160.557866c22.001237-9.92614 39.909221-26.196822 51.984319-47.788733a39.499896 39.499896 0 0 0-15.247369-53.519289l-0.511657-0.511657zM0 285.024493c0-44.616462 25.173509-83.911695 65.594386-102.331336L410.860313 22.03296a240.171645 240.171645 0 0 1 200.978743 0L957.821302 182.1815c40.420878 18.931297 65.594386 58.328861 65.594386 102.331336 0 44.104806-25.173509 84.014027-65.594386 102.331336l-345.879915 160.14854a240.478639 240.478639 0 0 1-201.4904 0L65.594386 387.867485a112.359807 112.359807 0 0 1-65.594386-102.331336v-0.511656z m78.795128 0c0 5.116567 1.53497 22.512894 19.85228 30.904063l345.879914 160.046209c42.979161 19.95461 91.791208 19.95461 134.872701 0L925.075274 315.928556a34.076335 34.076335 0 0 0 0-61.910458L579.297691 93.971889a160.046209 160.046209 0 0 0-134.258712 0L98.647408 254.018098a34.076335 34.076335 0 0 0-19.954611 31.006395z" fill="#959A9F" p-id="9467">
            </path>
          </svg>
          <span class="tool-label" v-if="showLabels">图层管理</span>
        </button>
        <!-- 航点地图 -->
        <button
          class="tool-button"
          :class="{ active: activeToolId === 'map' }"
          @click="selectTool('map')"
          title="航点地图">
          <svg width="20" height="20" viewBox="0 0 1024 1024" version="1.1" xmlns="http://www.w3.org/2000/svg">
            <path d="M946.545954 170.653705c-7.391341-6.209422-17.210987-8.839319-26.731828-7.125281l-237.180887 42.196031-60.578704-20.139689c-17.274432-5.750981-35.678594 3.578502-41.386596 20.72502-5.687536 17.146519 3.578502 35.667338 20.72502 41.365106l68.460208 22.759353c1.589195 0.525979 3.215228 0.915859 4.856611 1.196245l0 265.596054-135.363967-41.666982c-11.523451-3.535523-23.707958 2.939959-27.26497 14.420431-3.535523 11.513218 2.918469 23.728424 14.44192 27.26497l148.187017 45.617967 0 273.828552L391.156748 777.050632l0-123.269511c0-12.045337-9.755178-21.811772-21.811772-21.811772s-21.811772 9.766434-21.811772 21.811772l0 123.82005-218.116693 34.229592 0.001023-616.0948c0-18.073634-14.654768-32.717146-32.717146-32.717146s-32.717146 14.644535-32.717146 32.717146L63.981197 850.087065c0 9.56382 4.175089 18.648732 11.460006 24.868387 7.263427 6.209422 16.912182 8.978489 26.327622 7.454786l271.048229-42.536792 303.979246 85.37239c2.87549 0.809435 5.857405 1.213641 8.839319 1.213641 1.959631 0 3.919263-0.181125 5.857405-0.532119l239.906975-43.655266c15.549138-2.822278 26.859741-16.380062 26.859741-32.185027l0.001023-654.351102C958.261787 186.054463 953.980273 176.87336 946.545954 170.653705zM892.826472 510.465041l-174.49315 30.369681 0-275.004331 174.49315-31.039947L892.826472 510.465041zM718.332299 854.541517l0-269.420149 174.49315-30.367635 0 268.035616L718.332299 854.541517z" fill="currentColor"/>
            <path d="M209.677487 365.927133c10.692526 21.193695 108.99439 159.743214 138.879024 201.704908 6.134721 8.616238 16.060791 13.738909 26.646893 13.738909 0.021489 0 0.063445 0 0.084934 0 10.629081-0.031722 20.555152-5.207605 26.668383-13.888312 29.77821-42.302455 127.717823-181.852768 137.771807-201.214744 13.20679-25.411763 20.470217-55.008848 20.470217-83.327824 0-102.231359-83.178421-185.399547-185.399547-185.399547-102.221126 0-185.399547 83.168188-185.399547 185.399547 0 28.297486 6.816243 56.201 20.277836 82.954317C209.677487 365.905644 209.677487 365.915877 209.677487 365.927133zM374.799198 162.975838c66.138326 0 119.964232 53.815673 119.964232 119.964232 0 17.754363-4.771677 37.137829-13.057387 53.102429-5.709025 10.564613-55.211463 82.411965-106.652042 155.865907-47.223534-66.808592-100.730169-143.639444-106.949824-155.471934-8.797363-17.530258-13.270235-35.529191-13.270235-53.496401C254.834966 216.791511 308.660872 162.975838 374.799198 162.975838z" fill="currentColor"/>
            <path d="M374.805338 282.986119m-54.528917 0a53.287 53.287 0 1 0 109.057835 0 53.287 53.287 0 1 0-109.057835 0Z" fill="currentColor"/>
          </svg>
          <span class="tool-label" v-if="showLabels">航点地图</span>
        </button>

        <!-- 帮助信息 -->
        <button
          class="tool-button"
          :class="{ active: activeToolId === 'help' }"
          @click="selectTool('help')"
          title="帮助信息">
          <svg width="20" height="20" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
            <path d="M12 22C17.5228 22 22 17.5228 22 12C22 6.47715 17.5228 2 12 2C6.47715 2 2 6.47715 2 12C2 17.5228 6.47715 22 12 22Z" stroke="currentColor" stroke-width="1.5" stroke-linecap="round" stroke-linejoin="round"/>
            <path d="M9.09009 9.00001C9.32519 8.33167 9.78924 7.76811 10.4 7.40914C11.0108 7.05016 11.729 6.91895 12.4273 7.03871C13.1255 7.15848 13.7588 7.52153 14.2151 8.06353C14.6714 8.60553 14.9211 9.29152 14.92 10C14.92 12 11.92 13 11.92 13" stroke="currentColor" stroke-width="1.5" stroke-linecap="round" stroke-linejoin="round"/>
            <path d="M12 17H12.01" stroke="currentColor" stroke-width="1.5" stroke-linecap="round" stroke-linejoin="round"/>
          </svg>
          <span class="tool-label" v-if="showLabels">帮助</span>
        </button>

        <!-- 版本信息 -->
        <button
          class="tool-button"
          :class="{ active: activeToolId === 'info' }"
          @click="selectTool('info')"
          title="版本信息">
          <svg width="20" height="20" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
            <path d="M12 22C17.5228 22 22 17.5228 22 12C22 6.47715 17.5228 2 12 2C6.47715 2 2 6.47715 2 12C2 17.5228 6.47715 22 12 22Z" stroke="currentColor" stroke-width="1.5" stroke-linecap="round" stroke-linejoin="round"/>
            <path d="M12 16V12" stroke="currentColor" stroke-width="1.5" stroke-linecap="round" stroke-linejoin="round"/>
            <path d="M12 8H12.01" stroke="currentColor" stroke-width="1.5" stroke-linecap="round" stroke-linejoin="round"/>
          </svg>
          <span class="tool-label" v-if="showLabels">版本</span>
        </button>
      </div>

      <div class="divider"></div>

      <div class="tool-group">
        <!-- 保存 -->
        <button
          class="tool-button"
          :class="{ active: activeToolId === 'save' }"
          @click="selectTool('save')"
          title="保存">
          <svg width="20" height="20" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
            <path d="M19 21H5C3.89543 21 3 20.1046 3 19V5C3 3.89543 3.89543 3 5 3H14L21 10V19C21 20.1046 20.1046 21 19 21Z" stroke="currentColor" stroke-width="1.5" stroke-linecap="round" stroke-linejoin="round"/>
            <path d="M17 21V13H7V21" stroke="currentColor" stroke-width="1.5" stroke-linecap="round" stroke-linejoin="round"/>
            <path d="M7 3V8H14" stroke="currentColor" stroke-width="1.5" stroke-linecap="round" stroke-linejoin="round"/>
          </svg>
          <span class="tool-label" v-if="showLabels">保存</span>
        </button>

        <!-- 导入 -->
        <button
          class="tool-button"
          :class="{ active: activeToolId === 'sync' }"
          @click="selectTool('sync')"
          title="导入">
          <svg width="20" height="20" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
            <path d="M21 12C21 16.9706 16.9706 21 12 21C7.02944 21 3 16.9706 3 12C3 7.02944 7.02944 3 12 3" stroke="currentColor" stroke-width="1.5" stroke-linecap="round" stroke-linejoin="round"/>
            <path d="M16 3L21 3L21 8" stroke="currentColor" stroke-width="1.5" stroke-linecap="round" stroke-linejoin="round"/>
            <path d="M21 3L14 10" stroke="currentColor" stroke-width="1.5" stroke-linecap="round" stroke-linejoin="round"/>
          </svg>
          <span class="tool-label" v-if="showLabels">导入</span>
        </button>
      </div>

      <!-- 切换工具栏布局按钮 -->
      <div class="tool-layout-toggle">
        <button
          class="toggle-button"
          @click="() => {
            showLabels = !showLabels;
          }"
          :title="showLabels ? '折叠工具栏' : '展开工具栏'">
          <svg width="16" height="16" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
            <path d="M4 6H20" stroke="currentColor" stroke-width="1.5" stroke-linecap="round" stroke-linejoin="round"/>
            <path d="M4 12H20" stroke="currentColor" stroke-width="1.5" stroke-linecap="round" stroke-linejoin="round"/>
            <path d="M4 18H20" stroke="currentColor" stroke-width="1.5" stroke-linecap="round" stroke-linejoin="round"/>
          </svg>
        </button>
      </div>
    </div>

    <!-- 添加连接线组件 -->
    <AddEdge
      :visible="activeToolId === 'renderNodeEdge'"
    />

    <!-- 添加节点组件 -->
    <AddNode
      :visible="activeToolId === 'renderTaskNode'"
    />

    <!-- 航点地图组件 -->
    <MapPage
      :visible="activeToolId === 'map'"
    />

    <!-- 图层管理组件 -->
    <LayerPanel
      :visible="activeToolId === 'Layer'"
      @close="selectTool('cursor')"
    />

  </div>
</template>

<script setup lang="ts">
import { ref } from 'vue';
import AddEdge from '@/components/ToolMenu/AddEdge.vue';
import AddNode from '@/components/ToolMenu/AddNode.vue';
import MapPage from '@/components/ToolMenu/MapPage.vue';
import LayerPanel from '@/components/ToolMenu/LayerPanel.vue';
import { activeToolId } from '@/types/Manger';
import { selectTool } from '@/types/ToolMenu';

// 响应式状态，使用导入的共享状态
const showLabels = ref<boolean>(false); // 默认显示标签

</script>

<style scoped>
.tool-menu {
  position: fixed;
  left: 16px;
  top: 50%;
  transform: translateY(-50%);
  display: flex;
  flex-direction: column;
  background-color: #ffffff;
  border-radius: 8px;
  box-shadow: 0 2px 12px rgba(0, 0, 0, 0.1);
  padding: 8px;
  gap: 8px;
  z-index: 1000;
  transition: all 0.3s ease;
}

.tool-group {
  display: flex;
  flex-direction: column;
  gap: 4px;
}

.tool-button {
  display: flex;
  align-items: center;
  gap: 8px;
  padding: 8px;
  border: none;
  background-color: transparent;
  border-radius: 6px;
  cursor: pointer;
  color: #64748B;
  transition: all 0.2s ease;
}

.tool-button:hover {
  background-color: #F1F5F9;
  color: #1E293B;
}

.tool-button.active {
  background-color: #EFF6FF;
  color: #3B82F6;
}

.tool-label {
  font-size: 12px;
  white-space: nowrap;
  transition: opacity 0.3s ease;
}

.divider {
  height: 1px;
  background-color: #E2E8F0;
  margin: 4px 0;
}

.tool-layout-toggle {
  display: flex;
  justify-content: center;
  margin-top: 8px;
}

.toggle-button {
  padding: 6px;
  border: none;
  background-color: transparent;
  color: #64748B;
  border-radius: 4px;
  cursor: pointer;
  transition: all 0.2s ease;
}

.toggle-button:hover {
  background-color: #F1F5F9;
  color: #1E293B;
}
</style>
