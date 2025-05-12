<template>
  <div class="map-overlay" v-if="visible" :class="{ visible: showMap }">
    <div class="map-container">
      <div class="map-image-div">
        <div class="map-content-wrapper" ref="mapContainer">
          <img class="map-image" ref="mapImage" src="/map/RUMC.png" alt="航点地图" @click="handleMapClick">
          <!-- 渲染所有航点 -->
          <div v-if="layers.find(layer => layer.name === '航点图层')?.visible">
            <div
              v-for="point in points"
              :key="point.id.value"
              class="map-point"
              :style="{
                left: `${point.position.x}px`,
                top: `${point.position.y}px`,
                backgroundColor: point.color.value,
                position: 'absolute'
              }"
              :class="{ active: selectedPoint && selectedPoint.id.value === point.id.value }"
              @click.stop="selectPointById(point.id.value)"
            >
              <div class="point-text" v-if="point.text.value" :style="{ fontSize: point.fontSize.value + 'px', color: point.textColor.value }">
                {{ point.text.value }}
              </div>

              <div class="point-waypoint-text" v-if="point.waypoint" :style="{ fontSize: point.fontSize.value + 'px', color: point.textColor.value }">
                {{ point.waypoint.x.toFixed(2) }}, {{ point.waypoint.y.toFixed(2) }}
              </div>
            </div>
          </div>
          <!-- 渲染地图设置点 -->
          <div v-if="layers.find(layer => layer.name === '地图设置图层')?.visible">
            <div
              v-for="point in MapSettingsPoints"
              :key="point.id.value"
              class="map-point"
              :style="{
                left: `${point.position.x}px`,
                top: `${point.position.y}px`,
                backgroundColor: point.color.value,
                position: 'absolute'
              }"
            >
              <div class="point-text" v-if="point.text.value" :style="{ fontSize: point.fontSize.value + 'px', color: point.textColor.value }">
                {{ point.text.value }}
              </div>
            </div>
          </div>
          <!-- 渲染预览点 -->
          <div
            v-if="previewPoints !== null"
            :key="previewPoints.id.value"
            class="map-point"
            :style="{ // 使用鼠标位置
              left: `${mousePosition.x + WindowWidth / 80}px`,
              top: `${mousePosition.y + WindowHeight / 40}px`,
              backgroundColor: previewPoints.color.value,
              position: 'absolute'
            }"
            :class="{ active: selectedPoint && selectedPoint.id.value === previewPoints.id.value }"
            @click.stop="selectPointById(previewPoints.id.value)"
          >
            <div class="point-text" v-if="previewPoints.text.value" :style="{ fontSize: previewPoints.fontSize.value + 'px', color: previewPoints.textColor.value }">
              {{ previewPoints.text.value }}
            </div>
          </div>
          <!-- 渲染区域对象-->
          <div v-if="layers.find(layer => layer.name === '区域图层')?.visible">
            <div
              v-for="area in areas"
              :key="area.id"
              class="area"
              :style="{
                position: 'absolute',
                left: `${area.leftTop.x}px`,
                top: `${area.leftTop.y}px`,
                width: `${area.rightBottom.x - area.leftTop.x}px`,
                height: `${area.rightBottom.y - area.leftTop.y}px`,
                backgroundColor: area.color,
                borderRadius: '8px',
                border: `2px solid ${area.color}`,
                zIndex: 0,
              }"
              @mousedown="handleAreaMouseDown($event, area)"
            >
            <div
              class="group-label"
              :style="{ backgroundColor: area.color }"
            >
                {{ area.name }} (#{{ area.id }})
              </div>
            </div>
          </div>
          <!-- 渲染机器人 -->
          <div
            v-if="RobotPose !== undefined"
            :key="RobotPose.id.value"
            class="map-point"
            :style="{ // 使用鼠标位置
              left: `${RobotPose.position.x}px`,
              top: `${RobotPose.position.y}px`,
              backgroundColor: RobotPose.color.value,
              position: 'absolute'
            }"
          >
            <div class="point-text" v-if="RobotPose.text.value" :style="{ fontSize: RobotPose.fontSize.value + 'px', color: RobotPose.textColor.value }">
              {{ RobotPose.text.value }}
            </div>
          </div>
        </div>
        <!-- 航点菜单 -->
        <div class="menu point-menu" id="pointMenu" v-show="showPointMenu" :style="pointMenuStyle">
          <div class="menu-item" id="pointColor">
            <button id="point-color-button" @click="toggleColorPicker">
              <div class="icon">
                <svg style="fill: #666" width="18" height="18" viewBox="0 0 1024 1024" version="1.1" xmlns="http://www.w3.org/2000/svg">
                  <path d="M512 512m-512 0a512 512 0 1 0 1024 0 512 512 0 1 0-1024 0Z"></path>
                </svg>
              </div>
              <div class="menu-text">颜色</div>
              <div class="arrow">
                <svg width="8" height="8" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                  <path d="m3.414 7.086-.707.707a1 1 0 0 0 0 1.414l7.778 7.778a2 2 0 0 0 2.829 0l7.778-7.778a1 1 0 0 0 0-1.414l-.707-.707a1 1 0 0 0-1.415 0l-7.07 7.07-7.072-7.07a1 1 0 0 0-1.414 0Z" fill="currentColor"></path>
                </svg>
              </div>
            </button>

            <div class="color-picker-content" v-show="showColorPicker">
              <div class="color-grid">
                <div
                  v-for="color in colorOptions"
                  :key="color"
                  class="color-swatch"
                  :class="{ selected: selectedColor === color }"
                  :style="{ backgroundColor: color }"
                  :data-color="color"
                  @click="selectColor(color)"
                ></div>
                <div class="color-option add" @click="toggleCustomColor">+</div>
              </div>

              <div class="custom-color-section" v-show="showCustomColor">
                <div class="custom-color-header">自定义颜色</div>
                <div class="custom-color-input">
                  <input type="color" v-model="customColor">
                  <input type="text" v-model="customColor">
                </div>
                <button id="addCustomColorButton" @click="addCustomColor">添加颜色</button>
              </div>
            </div>
          </div>

          <div class="menu-item" id="textProperties">
            <button id="textProperties-button" @click="toggleTextProperties">
              <div class="icon">
                <svg style="fill: #666" width="20" height="20" viewBox="0 0 1024 1024" version="1.1" xmlns="http://www.w3.org/2000/svg">
                  <path d="M747.24 345.95c-11.47 0-20.76-9.3-20.76-20.76v-20.76H297.51v20.76c0 11.46-9.29 20.76-20.76 20.76S256 336.65 256 325.19v-41.51c0-11.46 9.29-20.76 20.76-20.76h470.49c11.47 0 20.76 9.3 20.76 20.76v41.51c-0.01 11.46-9.3 20.76-20.77 20.76z"></path>
                  <path d="M512 761.08c-11.47 0-20.76-9.3-20.76-20.76V283.68c0-11.46 9.29-20.76 20.76-20.76 11.47 0 20.76 9.3 20.76 20.76v456.65c0 11.45-9.29 20.75-20.76 20.75z"></path>
                  <path d="M581.19 761.08H442.81c-11.47 0-20.76-9.3-20.76-20.76s9.29-20.76 20.76-20.76h138.38c11.47 0 20.76 9.3 20.76 20.76s-9.29 20.76-20.76 20.76z"></path>
                </svg>
              </div>
              <div class="menu-text">文本</div>
              <div class="arrow">
                <svg width="8" height="8" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                  <path d="m3.414 7.086-.707.707a1 1 0 0 0 0 1.414l7.778 7.778a2 2 0 0 0 2.829 0l7.778-7.778a1 1 0 0 0 0-1.414l-.707-.707a1 1 0 0 0-1.415 0l-7.07 7.07-7.072-7.07a1 1 0 0 0-1.414 0Z" fill="currentColor"></path>
                </svg>
              </div>
            </button>

            <div class="text-properties-content" v-show="showTextProperties">
              <div class="text-input-section">
                <div class="text-input-label">文本内容</div>
                <textarea class="text-input-area" v-model="pointText" rows="3" placeholder="请输入文本"></textarea>
              </div>

              <div class="text-style-section">
                <div class="text-size-control">
                  <div class="control-label">字体大小</div>
                  <div class="size-input-group">
                    <input type="number" class="font-size" v-model="fontSize" min="8" max="72">
                    <span class="unit">px</span>
                  </div>
                </div>

                <div class="text-color-control">
                  <div class="control-label">文本颜色</div>
                  <div class="color-grid text-color-grid">
                    <div
                      v-for="color in textColorOptions"
                      :key="color"
                      class="color-option"
                      :class="{ selected: selectedTextColor === color }"
                      :style="{ backgroundColor: color }"
                      :data-color="color"
                      @click="selectTextColor(color)"
                    ></div>
                  </div>
                </div>
              </div>
            </div>
          </div>

          <div class="menu-item" id="pointDelete">
            <button id="point-delete-button" @click="toggleDeleteConfirm">
              <div class="icon">
                <svg style="fill: #666" width="18" height="18" viewBox="0 0 1024 1024" version="1.1" xmlns="http://www.w3.org/2000/svg">
                  <path d="M834.24768 256c-14.15168 0-25.6 11.08992-25.6 24.79616v668.80512c0 13.66016-11.47392 24.80128-25.6 24.80128H226.74944c-14.12608 0-25.6-11.14112-25.6-24.80128V280.79616c0-13.70624-11.44832-24.79616-25.6-24.79616s-25.6 11.08992-25.6 24.79616v668.80512c0 41.02656 34.45248 74.39872 76.8 74.39872h556.29824c42.34752 0 76.8-33.37216 76.8-74.39872V280.79616c0-13.70624-11.44832-24.79616-25.6-24.79616zM336.21504 51.2h353.28a25.6 25.6 0 0 0 0-51.2h-353.28a25.6 25.6 0 0 0 0 51.2z"></path>
                  <path d="M433.06496 846.99136v-558.08a25.6 25.6 0 0 0-51.2 0v558.08a25.6 25.6 0 0 0 51.2 0zM636.3392 846.99136v-558.08a25.6 25.6 0 0 0-51.2 0v558.08a25.6 25.6 0 0 0 51.2 0zM970.24 124.58496h-916.48a25.6 25.6 0 0 0 0 51.2h916.48a25.6 25.6 0 0 0 0-51.2z"></path>
                </svg>
              </div>
              <div class="menu-text">删除</div>
              <div class="arrow">
                <svg width="8" height="8" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                  <path d="m3.414 7.086-.707.707a1 1 0 0 0 0 1.414l7.778 7.778a2 2 0 0 0 2.829 0l7.778-7.778a1 1 0 0 0 0-1.414l-.707-.707a1 1 0 0 0-1.415 0l-7.07 7.07-7.072-7.07a1 1 0 0 0-1.414 0Z" fill="currentColor"></path>
                </svg>
              </div>
            </button>

            <div class="delete-confirm-content" v-show="showDeleteConfirm">
              <div class="delete-message">
                <div class="warning-icon">
                  <svg width="20" height="20" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                    <path d="M12 22c5.523 0 10-4.477 10-10S17.523 2 12 2 2 6.477 2 12s4.477 10 10 10zm0-7a1 1 0 1 0 0 2 1 1 0 0 0 0-2zm0-8a1 1 0 0 0-1 1v5a1 1 0 1 0 2 0V8a1 1 0 0 0-1-1z" fill="#f44336"></path>
                  </svg>
                </div>
                <div class="message-text">
                  <div class="message-title">删除航点</div>
                  <div class="message-desc">此操作无法撤销</div>
                </div>
              </div>
              <button class="confirm-button" id="deletePointButton" @click="deletePoint">删除</button>
            </div>
          </div>
        </div>

        <!-- 区域菜单 -->
        <div class="menu area-menu" id="areaMenu" v-show="showAreaMenu" :style="areaMenuStyle">
          <!-- 区域名称编辑 -->
          <div class="menu-item">
            <div class="menu-section">
              <div class="section-title">区域名称</div>
              <div class="input-row">
                <input type="text" class="name-input" v-model="areaName" placeholder="输入区域名称" @change="updateAreaName">
              </div>
            </div>
          </div>

          <!-- 区域颜色选择 -->
          <div class="menu-item" id="areaColor">
            <button id="area-color-button" @click="toggleAreaColorPicker">
              <div class="icon">
                <div class="color-preview" :style="{ backgroundColor: areaColor }"></div>
              </div>
              <div class="menu-text">颜色与透明度</div>
              <div class="arrow">
                <svg width="8" height="8" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                  <path d="m3.414 7.086-.707.707a1 1 0 0 0 0 1.414l7.778 7.778a2 2 0 0 0 2.829 0l7.778-7.778a1 1 0 0 0 0-1.414l-.707-.707a1 1 0 0 0-1.415 0l-7.07 7.07-7.072-7.07a1 1 0 0 0-1.414 0Z" fill="currentColor"></path>
                </svg>
              </div>
            </button>

            <div class="color-picker-content" v-show="showAreaColorPicker">
              <div class="color-grid">
                <div
                  v-for="color in areaColorOptions"
                  :key="color"
                  class="color-swatch"
                  :class="{ selected: areaColor === color }"
                  :style="{ backgroundColor: color }"
                  @click="selectAreaColor(color)"
                ></div>
              </div>
            </div>
          </div>

          <!-- 区域删除 -->
          <div class="menu-item" id="areaDelete">
            <button id="area-delete-button" @click="toggleAreaDeleteConfirm">
              <div class="icon">
                <svg style="fill: #666" width="18" height="18" viewBox="0 0 1024 1024" version="1.1" xmlns="http://www.w3.org/2000/svg">
                  <path d="M834.24768 256c-14.15168 0-25.6 11.08992-25.6 24.79616v668.80512c0 13.66016-11.47392 24.80128-25.6 24.80128H226.74944c-14.12608 0-25.6-11.14112-25.6-24.80128V280.79616c0-13.70624-11.44832-24.79616-25.6-24.79616s-25.6 11.08992-25.6 24.79616v668.80512c0 41.02656 34.45248 74.39872 76.8 74.39872h556.29824c42.34752 0 76.8-33.37216 76.8-74.39872V280.79616c0-13.70624-11.44832-24.79616-25.6-24.79616zM336.21504 51.2h353.28a25.6 25.6 0 0 0 0-51.2h-353.28a25.6 25.6 0 0 0 0 51.2z"></path>
                  <path d="M433.06496 846.99136v-558.08a25.6 25.6 0 0 0-51.2 0v558.08a25.6 25.6 0 0 0 51.2 0zM636.3392 846.99136v-558.08a25.6 25.6 0 0 0-51.2 0v558.08a25.6 25.6 0 0 0 51.2 0zM970.24 124.58496h-916.48a25.6 25.6 0 0 0 0 51.2h916.48a25.6 25.6 0 0 0 0-51.2z"></path>
                </svg>
              </div>
              <div class="menu-text">删除区域</div>
              <div class="arrow">
                <svg width="8" height="8" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                  <path d="m3.414 7.086-.707.707a1 1 0 0 0 0 1.414l7.778 7.778a2 2 0 0 0 2.829 0l7.778-7.778a1 1 0 0 0 0-1.414l-.707-.707a1 1 0 0 0-1.415 0l-7.07 7.07-7.072-7.07a1 1 0 0 0-1.414 0Z" fill="currentColor"></path>
                </svg>
              </div>
            </button>

            <div class="delete-confirm-content" v-show="showAreaDeleteConfirm">
              <div class="delete-message">
                <div class="warning-icon">
                  <svg width="20" height="20" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                    <path d="M12 22c5.523 0 10-4.477 10-10S17.523 2 12 2 2 6.477 2 12s4.477 10 10 10zm0-7a1 1 0 1 0 0 2 1 1 0 0 0 0-2zm0-8a1 1 0 0 0-1 1v5a1 1 0 1 0 2 0V8a1 1 0 0 0-1-1z" fill="#f44336"></path>
                  </svg>
                </div>
                <div class="message-text">
                  <div class="message-title">删除区域</div>
                  <div class="message-desc">此操作无法撤销</div>
                </div>
              </div>
              <button class="confirm-button" id="deleteAreaButton" @click="deleteArea">删除</button>
            </div>
          </div>
        </div>

        <!-- 悬浮功能菜单 -->
        <div
          class="floating-menu"
          :style="{ top: menuPosition.top + 'px', left: menuPosition.left + 'px' }"
          v-show="showMapTools"
        >
          <div
            class="floating-menu-header"
          >
            <div class="header-title">
              <svg viewBox="0 0 24 24" width="16" height="16" class="header-icon">
                <path d="M12 2L4.5 20.29l.71.71L12 18l6.79 3 .71-.71z" fill="currentColor"/>
              </svg>
              <span>地图工具</span>
            </div>
            <button class="floating-menu-toggle" :class="{ collapsed: !showFloatingMenu }" @click="()=>{showFloatingMenu = !showFloatingMenu}" title="折叠/展开菜单">
              <svg viewBox="0 0 24 24" width="14" height="14">
                <path d="M7.41 8.59L12 13.17l4.59-4.58L18 10l-6 6-6-6 1.41-1.41z"></path>
              </svg>
            </button>
          </div>

          <div class="floating-menu-content" v-show="showFloatingMenu">
            <!-- 标签页按钮 -->
            <div class="tab-headers">
              <button
                class="tab-button"
                :class="{ active: activeTab === 0 }"
                @click="activeTab = 0"
              >
                初始化地图
              </button>
              <button
                class="tab-button"
                :class="{ active: activeTab === 1 }"
                @click="activeTab = 1"
              >
                创建对象
              </button>
            </div>

            <!-- 第一个标签页：初始化地图 -->
            <div class="tab-content" v-show="activeTab === 0">
              <!-- 原有的初始化启动区 -->
              <div class="menu-section">
                <div class="section-title">初始化启动区</div>
                <button class="tool-button" @click="setStartingArea" data-tooltip="设置机器人启动区域">
                  <div class="button-icon">
                    <svg viewBox="0 0 24 24" width="16" height="16">
                      <path d="M12 2L4.5 20.29l.71.71L12 18l6.79 3 .71-.71z" fill="currentColor"></path>
                    </svg>
                  </div>
                  <span>放置启动区</span>
                </button>
              </div>

              <!-- 原有的边界点 -->
              <div class="menu-section">
                <div class="section-title">边界点</div>
                <div class="buttons-row">
                  <button class="tool-button boundary-button" @click="setTopLeftCorner" data-tooltip="放置地图左上角边界点">
                    <div class="button-icon">
                      <svg viewBox="0 0 24 24" width="16" height="16">
                        <path d="M4 4h6v6H4V4z M4 4L10 4 10 10 4 10z" fill="currentColor"></path>
                      </svg>
                    </div>
                    <span>左上角</span>
                  </button>
                  <button class="tool-button boundary-button" @click="setBottomRightCorner" data-tooltip="放置地图右下角边界点">
                    <div class="button-icon">
                      <svg viewBox="0 0 24 24" width="16" height="16">
                        <path d="M14 14h6v6h-6v-6z M14 14L20 14 20 20 14 20z" fill="currentColor"></path>
                      </svg>
                    </div>
                    <span>右下角</span>
                  </button>
                </div>
              </div>

              <!-- 原有的地图尺寸 -->
              <div class="menu-section">
                <div class="section-title">地图尺寸</div>
                <div class="input-row">
                  <div class="input-group">
                    <label for="map-width">长度 (m)</label>
                    <input type="number" id="map-width" class="size-input" v-model="mapWidth" min="1" step="1">
                  </div>
                  <div class="input-group">
                    <label for="map-height">宽度 (m)</label>
                    <input type="number" id="map-height" class="size-input" v-model="mapHeight" min="1" step="1">
                  </div>
                </div>
                <button class="tool-button apply-button" @click="applyMapSize" data-tooltip="应用尺寸设置到地图">
                  <div class="button-icon">
                    <svg viewBox="0 0 24 24" width="16" height="16">
                      <path d="M9 16.17L4.83 12l-1.42 1.41L9 19 21 7l-1.41-1.41L9 16.17z" fill="currentColor"></path>
                    </svg>
                  </div>
                  <span>重新计算航点</span>
                </button>
              </div>
            </div>

            <!-- 第二个标签页：创建对象 -->
            <div class="tab-content" v-show="activeTab === 1">
              <!-- 创建航点对象 -->
              <div class="menu-section">
                <div class="section-title">创建航点</div>
                <button class="tool-button" @click="createWaypoint" data-tooltip="在地图上创建新航点">
                  <div class="button-icon">
                    <svg viewBox="0 0 24 24" width="16" height="16">
                      <path d="M12 2C8.13 2 5 5.13 5 9c0 5.25 7 13 7 13s7-7.75 7-13c0-3.87-3.13-7-7-7zm0 9.5c-1.38 0-2.5-1.12-2.5-2.5s1.12-2.5 2.5-2.5 2.5 1.12 2.5 2.5-1.12 2.5-2.5 2.5z" fill="currentColor"></path>
                    </svg>
                  </div>
                  <span>添加航点</span>
                </button>
              </div>

              <!-- 创建区域 -->
              <div class="menu-section">
                <div class="section-title">创建区域</div>
                <div class="description-text">点击按钮后，在地图上依次选择左上角和右下角顶点来创建区域</div>
                <button class="tool-button" @click="createArea" data-tooltip="创建一个矩形区域">
                  <div class="button-icon">
                    <svg t="1746547363258" viewBox="0 0 1024 1024" version="1.1" xmlns="http://www.w3.org/2000/svg" p-id="18517" width="24" height="24">
                      <path fill="currentColor" d="M333.226667 197.290667l16.725333-83.669334 376.533333 75.178667-16.682666 83.669333zM790.698667 351.872l84.736-10.154667 40.533333 338.901334-84.736 10.154666zM227.285333 831.530667l559.189334-66.816 10.154666 84.736-559.232 66.816zM111.530667 782.08L196.224 225.365333l84.352 12.8-84.693333 556.8z" p-id="18518"></path>
                      <path fill="currentColor" d="M247.466667 277.333333C170.666667 277.333333 106.666667 213.333333 106.666667 136.533333S170.666667 0 247.466667 0 384 64 384 136.533333 324.266667 277.333333 247.466667 277.333333z m0-192c-29.866667 0-55.466667 25.6-55.466667 51.2s25.6 51.2 51.2 51.2S298.666667 166.4 298.666667 136.533333 277.333333 85.333333 247.466667 85.333333zM136.533333 1024C64 1024 0 960 0 887.466667s64-136.533333 136.533333-136.533334c76.8 0 136.533333 64 136.533334 136.533334S213.333333 1024 136.533333 1024z m0-192c-29.866667 0-51.2 25.6-51.2 51.2S110.933333 938.666667 136.533333 938.666667s51.2-25.6 51.2-51.2-21.333333-55.466667-51.2-55.466667zM810.666667 392.533333c-76.8 0-136.533333-64-136.533334-136.533333s64-136.533333 136.533334-136.533333 136.533333 64 136.533333 136.533333-59.733333 136.533333-136.533333 136.533333z m0-192c-29.866667 0-51.2 25.6-51.2 51.2s25.6 51.2 51.2 51.2 51.2-25.6 51.2-51.2-21.333333-51.2-51.2-51.2zM887.466667 917.333333c-76.8 0-136.533333-64-136.533334-136.533333S810.666667 640 887.466667 640s136.533333 64 136.533333 136.533333-64 140.8-136.533333 140.8z m0-192c-29.866667 0-51.2 25.6-51.2 51.2s25.6 51.2 51.2 51.2S938.666667 810.666667 938.666667 780.8s-25.6-55.466667-51.2-55.466667z" p-id="18519"></path>
                    </svg>
                  </div>
                  <span>创建区域</span>
                </button>
              </div>
            </div>
          </div>
        </div>
      </div>

      <!-- 关闭按钮 -->
      <button class="close-button" @click="closeMap">
        <svg viewBox="0 0 24 24" width="14" height="14">
          <path d="M19 6.41L17.59 5 12 10.59 6.41 5 5 6.41 10.59 12 5 17.59 6.41 19 12 13.41 17.59 19 19 17.59 13.41 12z"></path>
        </svg>
      </button>

      <!-- 状态栏 -->
      <div class="map-status-bar">
        <div class="status-section">
          <div class="status-icon">
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
              <path d="M12 22C17.5228 22 22 17.5228 22 12C22 6.47715 17.5228 2 12 2C6.47715 2 2 6.47715 2 12C2 17.5228 6.47715 22 12 22Z" stroke="currentColor" stroke-width="1.5" stroke-linecap="round" stroke-linejoin="round"/>
              <path d="M12 16V12" stroke="currentColor" stroke-width="1.5" stroke-linecap="round" stroke-linejoin="round"/>
              <path d="M12 8H12.01" stroke="currentColor" stroke-width="1.5" stroke-linecap="round" stroke-linejoin="round"/>
            </svg>
          </div>
          <div class="status-text">左键选择航点 | Ctrl+左键创建航点</div>
        </div>

        <div class="status-section" v-if="selectedPoint">
          <div class="status-text">当前选中航点 ID: {{ selectedPoint.id.value }}</div>
        </div>

        <div class="status-section map-coordinates" v-if="mousePosition.x && mousePosition.y">
          <div class="status-text">坐标: {{ mousePosition.x }}, {{ mousePosition.y }}</div>
        </div>
      </div>

      <!-- 区域创建提示 -->
      <div class="area-creation-hint" v-if="isCreatingArea">
        <div class="hint-icon">
          <svg viewBox="0 0 24 24" width="20" height="20">
            <path d="M12 2C6.48 2 2 6.48 2 12s4.48 10 10 10 10-4.48 10-10S17.52 2 12 2zm1 15h-2v-2h2v2zm0-4h-2V7h2v6z" fill="currentColor"></path>
          </svg>
        </div>
        <div v-if="!areaFirstPoint">请点击选择区域的左上角点</div>
        <div v-else>请点击选择区域的右下角点</div>
      </div>
    </div>
  </div>
</template>

<script setup lang="ts" name="MapPage">
import { onMounted, defineProps, watch, onUnmounted, watchEffect } from 'vue';
import { selectedPoint, pointText, fontSize, selectedTextColor } from '@/types/extensions/ToolMenu/PointMenu';
import {
  closeMap,
  showMap,
  handleMapClick,
  pointMenuStyle,
  showPointMenu,
  mapContainer
} from '@/types/extensions/ToolMenu/MapPage';
import {
  toggleColorPicker,
  toggleCustomColor,
  selectColor,
  addCustomColor,
  toggleTextProperties,
  selectTextColor,
  toggleDeleteConfirm,
  deletePoint,
  selectPointById,
  colorOptions,
  showColorPicker,
  showCustomColor,
  showTextProperties,
  showDeleteConfirm,
  textColorOptions,
  selectedColor,
  customColor
} from '@/types/extensions/ToolMenu/PointMenu';
import {
  showFloatingMenu,
  showMapTools,
  menuPosition,
  setStartingArea,
  setTopLeftCorner,
  setBottomRightCorner,
  applyMapSize,
  previewPoints,
  mousePosition,
  createArea,
  isCreatingArea,
  areaFirstPoint,
  activeTab,
  showAreaMenu,
  areaMenuStyle,
  selectedArea,
  areaName,
  areaColor,
  createWaypoint,
  mapImage
} from '@/types/extensions/ToolMenu/MapPage';
import {
  showAreaColorPicker,
  showAreaDeleteConfirm,
  updateAreaMenuPosition,
  handleOutsideClick,
  toggleAreaColorPicker,
  toggleAreaDeleteConfirm,
  selectAreaColor,
  updateAreaName,
  deleteArea,
  areaColorOptions
} from '@/types/extensions/ToolMenu/AreaMenu';
import { points } from '@/types/Manger';
import { mapWidth, mapHeight, MapSettingsPoints, areas } from '@/types/Manger';
import { type Area } from '@/types/Area';
import { layers } from '@/types/Layers';
import { RobotPose } from '@/types/Manger';
// 定义 props
const props = defineProps<{
  visible: boolean;
}>();

const WindowWidth = window.innerWidth;
const WindowHeight = window.innerHeight;


// 监听 visible 属性变化
watch(() => props.visible, (newVal) => {
  if (newVal) {
    setTimeout(() => {
      showMap.value = true;
    }, 50);
  } else {
    showMap.value = false;
  }
});

const handleAreaMouseDown = (event: MouseEvent, area: Area) => {
  // 防止冒泡到地图
  event.stopPropagation();

  // 设置选中区域
  selectedArea.value = area;

  // 初始化表单值
  areaName.value = area.name;
  areaColor.value = area.color;

  // 重置菜单状态
  showAreaColorPicker.value = false;
  showAreaDeleteConfirm.value = false;

  // 计算区域的中心点坐标，而不是使用鼠标点击位置
  // 这样可以确保菜单出现在区域附近的一致位置
  const areaWidth = area.rightBottom.x - area.leftTop.x;

  // 获取点击点相对于容器的绝对坐标
  const containerRect = mapContainer.value?.getBoundingClientRect();
  if (!containerRect) return;

  // 优先放置在区域右侧，如果空间不足则放在左侧
  const areaCenterX = area.leftTop.x + areaWidth / 2 + containerRect.left;
  const areaCenterY = area.leftTop.y + containerRect.top;

  // 显示区域菜单
  setTimeout(() => {
    showAreaMenu.value = true;
  }, 200);

  // 更新菜单位置，使用经过计算的位置，而不是鼠标点击位置
  updateAreaMenuPosition(areaCenterX, areaCenterY);

  // 关闭其他菜单
  showPointMenu.value = false;
};

// 跟踪鼠标位置
const trackMousePosition = (event: MouseEvent) => {
  if (!mapContainer.value) return;

  const rect = mapContainer.value.getBoundingClientRect();
  mousePosition.value = {
    x: Math.round(event.clientX - rect.left),
    y: Math.round(event.clientY - rect.top)
  };
};

// 处理右键菜单和鼠标位置跟踪
onMounted(() => {
  const stop = watchEffect(() => {
    const container = mapContainer.value;
    if (container) {
      container.addEventListener('contextmenu', (e) => {
        e.preventDefault();
      });
      container.addEventListener('mousemove', trackMousePosition);

      // 添加全局点击事件监听
      document.addEventListener('click', handleOutsideClick);

      stop();
    }
  });
});

// 移除事件监听器
onUnmounted(() => {
  if (mapContainer.value) {
    mapContainer.value.removeEventListener('mousemove', trackMousePosition);
  }

  // 移除全局点击事件监听
  document.removeEventListener('click', handleOutsideClick);
});
</script>

<style scoped>
.map-overlay {
  position: fixed;
  top: 0;
  left: 0;
  width: 100%;
  height: 100%;
  background-color: rgba(0, 0, 0, 0.5);
  display: flex;
  justify-content: center;
  align-items: center;
  z-index: 2000;
  opacity: 0;
  transition: opacity 0.3s ease;
}

.map-overlay.visible {
  opacity: 1;
}

.map-container {
  position: relative;
  width: 85%;
  max-width: 1400px;
  height: 85%;
  background-color: #f8fafc;
  border-radius: 12px;
  overflow: visible; /* 确保弹出框不被裁剪 */
  box-shadow: 0 8px 24px rgba(0, 0, 0, 0.08);
  display: flex;
  flex-direction: column;
}

.map-image-div {
  position: relative;
  flex: 1;
  background-color: #fff;
  overflow: hidden; /* 仅图像区域的滚动条隐藏 */
  border-radius: 12px 12px 0 0; /* 保持顶部圆角 */
  box-shadow: inset 0 0 10px rgba(0, 0, 0, 0.05); /* 内阴影效果 */
  display: flex;
  justify-content: center;
  align-items: center;
}

.map-content-wrapper {
  position: relative;
  display: inline-block; /* 关键：使容器尺寸依据图片内容 */
  max-width: 100%;
  max-height: 100%;
  transform-origin: center; /* 确保从中心点缩放 */
}

.map-image {
  display: block;
  max-width: 100%;
  max-height: 100%;
  width: auto;
  height: auto;
  object-fit: contain;
  cursor: crosshair;
}

/* 航点样式 */
.map-point {
  position: absolute;
  width: 12px;
  height: 12px;
  border-radius: 50%;
  transform: translate(-50%, -50%);
  cursor: pointer;
  z-index: 10;
  box-shadow: 0 0 0 2px white;
  transition: transform 0.2s ease;
}

/* 增加航点点击区域，提升用户体验 */
.map-point::before {
  content: '';
  position: absolute;
  width: 24px;
  height: 24px;
  border-radius: 50%;
  top: 50%;
  left: 50%;
  transform: translate(-50%, -50%);
  z-index: -1;
}

.map-point.active {
  transform: translate(-50%, -50%) scale(1.5);
  z-index: 11;
}

.point-text {
  position: absolute;
  top: -20px;
  left: 50%;
  transform: translateX(-50%);
  white-space: nowrap;
  background-color: rgba(255, 255, 255, 0.8);
  padding: 2px 6px;
  border-radius: 4px;
  box-shadow: 0 1px 3px rgba(0, 0, 0, 0.1);
  pointer-events: none;
  z-index: 12;
}

.point-waypoint-text {
  position: absolute;
  top: 10px;
  left: 50%;
  transform: translateX(-50%);
  white-space: nowrap;
  background-color: rgba(255, 255, 255, 0.4);
  padding: 2px 6px;
  border-radius: 4px;
  box-shadow: 0 1px 3px rgba(0, 0, 0, 0.1);
  pointer-events: none;
  z-index: 12;
}

/* 关闭按钮样式 */
.close-button {
  position: absolute;
  top: -18px;
  right: -18px;
  background-color: #f43f5e; /* 使用醒目的红色 */
  color: white;
  border: 2px solid white; /* 白色边框增加凸显效果 */
  border-radius: 50%;
  width: 36px;
  height: 36px;
  display: flex;
  align-items: center;
  justify-content: center;
  cursor: pointer;
  box-shadow: 0 3px 12px rgba(0, 0, 0, 0.2); /* 增强阴影 */
  transition: all 0.2s ease;
  z-index: 2001;
  padding: 0;
  transform: translate(0, 0);
}

.close-button:hover {
  background-color: #e11d48; /* 深红色悬停效果 */
  transform: scale(1.05);
}

.close-button svg {
  width: 18px;
  height: 18px;
  fill: white; /* 白色图标 */
}

/* 状态栏样式 */
.map-status-bar {
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: 0 16px;
  height: 38px;
  background-color: #f8fafc;
  border-top: 1px solid #e2e8f0;
  border-radius: 0 0 12px 12px;
  font-size: 13px;
  color: #64748b;
  flex-wrap: wrap;
  box-shadow: 0 -1px 2px rgba(0, 0, 0, 0.03);
}

.status-section {
  display: flex;
  align-items: center;
  gap: 4px;
  white-space: nowrap;
  overflow: hidden;
  text-overflow: ellipsis;
  max-width: 50%;
  font-weight: 500;
}

.map-coordinates {
  max-width: 150px;
  overflow: hidden;
  text-overflow: ellipsis;
}

.map-status-icon {
  color: #3b82f6;
  font-size: 16px;
  margin-right: 4px;
}

/* 点菜单样式 */
.menu {
  position: absolute;
  background-color: #ffffff;
  border-radius: 8px;
  box-shadow: 0 4px 20px rgba(0, 0, 0, 0.15);
  padding: 10px;
  min-width: 250px;
  max-width: 280px;
  z-index: 1000;
  overflow: visible;
  border: 1px solid #e2e8f0;
}

.menu.point-menu {
  width: 280px; /* 设置固定宽度与子菜单一致 */
  max-width: 280px;
  min-width: 280px; /* 确保最小宽度也一致 */
  box-sizing: border-box; /* 确保padding和border计入宽度 */
}

.menu-item {
  position: relative;
  margin-bottom: 6px;
  overflow: visible; /* 确保子菜单可见 */
}

.menu-item:last-child {
  margin-bottom: 0;
}

.menu-item button {
  display: flex;
  align-items: center;
  width: 100%;
  padding: 10px 12px;
  background-color: transparent;
  border: none;
  border-radius: 6px;
  cursor: pointer;
  transition: background-color 0.2s;
  font-weight: 500;
}

.menu-item button:hover {
  background-color: #f1f5f9;
}

.icon {
  display: flex;
  align-items: center;
  justify-content: center;
  width: 24px;
  height: 24px;
  margin-right: 12px;
}

.menu-text {
  flex: 1;
  text-align: left;
  font-size: 14px;
  color: #334155;
}

.arrow {
  display: flex;
  align-items: center;
}

.color-picker-content,
.text-properties-content,
.delete-confirm-content {
  background-color: #ffffff;
  border-radius: 8px;
  box-shadow: 0 4px 16px rgba(0, 0, 0, 0.12);
  padding: 16px;
  margin-top: 8px;
  width: 100%;
  max-width: 100%;
  box-sizing: border-box;
  z-index: 1010;
  transform-origin: top center;
  transition: all 0.2s ease-out;
}

.color-picker-content {
  border: 1px solid #e2e8f0;
  padding: 12px;
}

.color-grid {
  display: grid;
  grid-template-columns: repeat(auto-fill, minmax(28px, 1fr));
  gap: 10px;
  margin-bottom: 12px;
  width: 100%;
  max-width: 100%;
}

.color-swatch {
  width: 28px;
  height: 28px;
  border-radius: 8px;
  cursor: pointer;
  transition: all 0.2s ease;
  border: 2px solid transparent;
  box-shadow: 0 1px 3px rgba(0, 0, 0, 0.1);
}

.color-swatch:hover {
  transform: scale(1.1);
  box-shadow: 0 2px 6px rgba(0, 0, 0, 0.15);
}

.color-swatch.selected {
  border: 2px solid #fff;
  box-shadow: 0 0 0 2px #3b82f6, 0 2px 5px rgba(0, 0, 0, 0.1);
}

.text-color-grid {
  grid-template-columns: repeat(5, 1fr);
}

.color-option {
  width: 24px;
  height: 24px;
  border-radius: 50%;
  cursor: pointer;
  border: 2px solid transparent;
  transition: all 0.2s;
}

.color-option:hover {
  transform: scale(1.1);
}

.color-option.selected {
  border-color: #3b82f6;
  transform: scale(1.1);
}

.color-option.add {
  display: flex;
  align-items: center;
  justify-content: center;
  background-color: #f8f9fa;
  color: #5f6368;
  font-weight: bold;
  font-size: 16px;
}

.custom-color-section {
  border-top: 1px solid #e2e8f0;
  padding-top: 12px;
  margin-top: 4px;
  width: 100%;
}

.custom-color-header {
  font-size: 14px;
  color: #64748b;
  margin-bottom: 10px;
  font-weight: 500; /* 稍微加粗标题 */
}

.custom-color-input {
  display: flex;
  align-items: center;
  gap: 8px;
  margin-bottom: 10px;
  width: 100%;
}

.custom-color-input input {
  flex-grow: 1;
  border: 1px solid #e2e8f0;
  border-radius: 4px;
  padding: 8px 10px;
  font-size: 14px;
  max-width: calc(100% - 40px);
}

.color-preview {
  width: 30px;
  height: 30px;
  border-radius: 6px;
  border: 1px solid #e2e8f0;
  box-shadow: 0 1px 3px rgba(0, 0, 0, 0.1);
}

.button {
  background-color: #3b82f6;
  color: white;
  border: none;
  border-radius: 4px;
  padding: 10px 12px; /* 增加按钮高度 */
  font-size: 14px;
  font-weight: 600; /* 加粗文字 */
  cursor: pointer;
  width: 100%;
  transition: all 0.2s ease;
  text-shadow: 0 1px 2px rgba(0, 0, 0, 0.2); /* 添加文字阴影增加可读性 */
  box-shadow: 0 1px 3px rgba(0, 0, 0, 0.1); /* 添加轻微阴影 */
  letter-spacing: 0.5px; /* 增加字间距 */
}

.button:hover {
  background-color: #2563eb;
  transform: translateY(-1px); /* 悬停时轻微上移 */
  box-shadow: 0 2px 5px rgba(0, 0, 0, 0.15); /* 增强悬停时的阴影 */
}

.text-input-section {
  margin-bottom: 12px;
  width: 100%;
}

.text-input-label {
  font-size: 14px;
  color: #64748b;
  margin-bottom: 6px;
}

.text-input-area {
  width: 100%;
  max-width: 100%; /* 确保不超过父容器 */
  padding: 8px;
  border: 1px solid #e2e8f0;
  border-radius: 4px;
  resize: vertical;
  font-size: 14px;
  transition: border-color 0.2s;
  box-sizing: border-box; /* 确保padding不会导致宽度超出 */
  min-height: 60px; /* 设置最小高度 */
  max-height: 120px; /* 设置最大高度 */
}

.text-input-area:focus {
  border-color: #3b82f6;
  outline: none;
}

.text-style-section {
  display: flex;
  flex-direction: column;
  gap: 12px;
  width: 100%;
}

.control-label {
  font-size: 14px;
  color: #64748b;
  margin-bottom: 6px;
}

.size-input-group {
  display: flex;
  align-items: center;
  gap: 6px;
}

.font-size {
  width: 60px;
  padding: 6px 8px;
  border: 1px solid #e2e8f0;
  border-radius: 4px;
  font-size: 14px;
  transition: border-color 0.2s;
}

.font-size:focus {
  border-color: #3b82f6;
  outline: none;
}

.unit {
  color: #64748b;
  font-size: 14px;
}

.delete-message {
  display: flex;
  align-items: flex-start;
  gap: 12px;
  margin-bottom: 16px; /* 增加与按钮的间距 */
}

.warning-icon {
  display: flex;
  align-items: center;
  justify-content: center;
}

.message-title {
  font-weight: 600; /* 加粗标题 */
  margin-bottom: 4px;
  color: #334155;
  font-size: 15px; /* 略微增大字号 */
}

.message-desc {
  font-size: 13px;
  color: #64748b;
  line-height: 1.4; /* 增加行高提高可读性 */
}

#deletePointButton {
  background-color: #ef4444;
  color: white;
  border: none;
  border-radius: 4px;
  padding: 10px 12px; /* 增加按钮高度 */
  font-size: 14px;
  font-weight: 600; /* 更加加粗文字 */
  cursor: pointer;
  width: 100%;
  transition: all 0.2s ease;
  text-shadow: 0 1px 2px rgba(0, 0, 0, 0.2); /* 增强文字阴影 */
  box-shadow: 0 1px 3px rgba(0, 0, 0, 0.1); /* 给按钮添加轻微阴影 */
  letter-spacing: 0.5px; /* 增加字间距 */
}

#deletePointButton:hover {
  background-color: #dc2626;
  transform: translateY(-1px); /* 悬停时轻微上移 */
  box-shadow: 0 2px 5px rgba(0, 0, 0, 0.15); /* 增强悬停时的阴影 */
}

#addCustomColorButton {
  background-color: #3b82f6;
  color: white;
  border: none;
  border-radius: 4px;
  padding: 10px 12px; /* 增加按钮高度 */
  font-size: 14px;
  font-weight: 600; /* 更加加粗文字 */
  cursor: pointer;
  width: 100%;
  transition: all 0.2s ease;
  text-shadow: 0 1px 2px rgba(0, 0, 0, 0.2); /* 增强文字阴影 */
  box-shadow: 0 1px 3px rgba(0, 0, 0, 0.1); /* 给按钮添加轻微阴影 */
  letter-spacing: 0.5px; /* 增加字间距 */
}

#addCustomColorButton:hover {
  background-color: #2563eb;
  transform: translateY(-1px); /* 悬停时轻微上移 */
  box-shadow: 0 2px 5px rgba(0, 0, 0, 0.15); /* 增强悬停时的阴影 */
}

.delete-confirm-content {
  border: 1px solid rgba(239, 68, 68, 0.2); /* 添加轻微的红色边框提示删除操作 */
}

.custom-color-input input[type="color"] {
  width: 32px;
  height: 32px;
  border: 1px solid #e2e8f0;
  padding: 0;
  background: none;
  cursor: pointer;
}

.custom-color-input input[type="text"] {
  flex: 1;
  padding: 6px 8px;
  border: 1px solid #e2e8f0;
  border-radius: 4px;
  font-size: 14px;
  max-width: calc(100% - 40px); /* 确保不超出父容器，留出颜色选择器的空间 */
  box-sizing: border-box;
}

/* 悬停在路径点上时的样式效果 */
.waypoint {
  position: absolute;
  width: 20px;
  height: 20px;
  border-radius: 50%;
  background-color: #3b82f6;
  cursor: pointer;
  transform-origin: center;
  transition: all 0.2s ease;
  display: flex;
  align-items: center;
  justify-content: center;
  font-size: 12px;
  color: white;
  font-weight: bold;
  box-shadow: 0 2px 4px rgba(0, 0, 0, 0.2);
  border: 2px solid white;
  user-select: none;
}

.waypoint:hover {
  transform: scale(1.1);
  box-shadow: 0 0 0 3px rgba(59, 130, 246, 0.3), 0 3px 6px rgba(0, 0, 0, 0.25);
  z-index: 1001;
}

.waypoint.selected {
  background-color: #2563eb;
  box-shadow: 0 0 0 4px rgba(59, 130, 246, 0.4), 0 4px 8px rgba(0, 0, 0, 0.3);
  transform: scale(1.2);
  z-index: 1002;
}

.selected-point-indicator {
  position: absolute;
  width: 30px;
  height: 30px;
  border-radius: 50%;
  border: 2px dashed #3b82f6;
  animation: pulse 1.5s infinite;
  opacity: 0.7;
  pointer-events: none;
}

@keyframes pulse {
  0% { transform: scale(1); opacity: 0.7; }
  50% { transform: scale(1.1); opacity: 0.5; }
  100% { transform: scale(1); opacity: 0.7; }
}

/* 悬浮菜单样式 */
.floating-menu {
  position: absolute;
  top: 20px;
  right: 20px;
  background-color: #ffffff;
  border-radius: 8px;
  box-shadow: 0 4px 20px rgba(0, 0, 0, 0.15);
  width: 260px;
  z-index: 1500;
  overflow: hidden;
  border: 1px solid #e2e8f0;
  transition: all 0.3s ease;
  max-height: calc(100% - 40px);
  overflow-y: auto;
}

.floating-menu-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 12px 16px;
  background-color: #f8fafc;
  border-bottom: 1px solid #e2e8f0;
  font-weight: 600;
  color: #334155;
  cursor: move;
  user-select: none;
  touch-action: none;
}

.header-title {
  display: flex;
  align-items: center;
  gap: 6px;
}

.header-icon {
  color: #3b82f6;
}

/* 拖动样式 */
.floating-menu.dragging {
  opacity: 0.9;
  box-shadow: 0 8px 30px rgba(0, 0, 0, 0.2);
}

.floating-menu-toggle {
  background: none;
  border: none;
  color: #64748b;
  cursor: pointer;
  padding: 4px;
  display: flex;
  align-items: center;
  justify-content: center;
  border-radius: 4px;
  transition: all 0.2s;
  width: 24px;
  height: 24px;
}

.floating-menu-toggle:hover {
  background-color: #e2e8f0;
  color: #334155;
}

.floating-menu-toggle svg {
  fill: currentColor;
  transition: transform 0.3s ease;
}

.floating-menu-toggle.collapsed svg {
  transform: rotate(-90deg);
}

.floating-menu-content {
  padding: 12px 16px;
  transform-origin: top center;
  transition: all 0.3s ease;
  max-height: 500px;
  opacity: 1;
}

.menu-section {
  margin-bottom: 16px;
  animation: fadeIn 0.3s ease-in-out;
}

@keyframes fadeIn {
  from { opacity: 0; transform: translateY(-10px); }
  to { opacity: 1; transform: translateY(0); }
}

.menu-section:last-child {
  margin-bottom: 0;
}

.section-title {
  font-size: 13px;
  font-weight: 600;
  color: #64748b;
  margin-bottom: 8px;
  display: flex;
  align-items: center;
}

.section-title::before {
  content: '';
  display: inline-block;
  width: 3px;
  height: 14px;
  background-color: #3b82f6;
  margin-right: 6px;
  border-radius: 3px;
}

.tool-button {
  display: flex;
  align-items: center;
  gap: 8px;
  width: 100%;
  padding: 8px 12px;
  background-color: #f8fafc;
  border: 1px solid #e2e8f0;
  border-radius: 6px;
  cursor: pointer;
  font-size: 14px;
  color: #334155;
  transition: all 0.2s ease;
  position: relative;
}

.tool-button:hover {
  background-color: #f1f5f9;
  transform: translateY(-1px);
  box-shadow: 0 2px 4px rgba(0, 0, 0, 0.05);
}

.tool-button:active {
  transform: translateY(0);
  box-shadow: none;
}

.tool-button::after {
  content: attr(data-tooltip);
  position: absolute;
  bottom: 100%;
  left: 50%;
  transform: translateX(-50%);
  background-color: rgba(51, 65, 85, 0.9);
  color: white;
  padding: 4px 8px;
  border-radius: 4px;
  font-size: 12px;
  white-space: nowrap;
  pointer-events: none;
  opacity: 0;
  transition: opacity 0.2s ease;
  z-index: 1600;
}

.tool-button:hover::after {
  opacity: 1;
}

.button-icon {
  display: flex;
  align-items: center;
  justify-content: center;
  width: 16px;
  height: 16px;
  color: #64748b;
}

.tool-button:hover .button-icon {
  color: #3b82f6;
}

.buttons-row {
  display: flex;
  gap: 8px;
}

.boundary-button {
  flex: 1;
}

.input-row {
  display: flex;
  gap: 8px;
  margin-bottom: 8px;
}

.input-group {
  flex: 1;
  display: flex;
  flex-direction: column;
  gap: 4px;
}

.input-group label {
  font-size: 12px;
  color: #64748b;
}

.size-input {
  width: 100%;
  padding: 6px 8px;
  border: 1px solid #e2e8f0;
  border-radius: 4px;
  font-size: 14px;
  background-color: #f8fafc;
  color: #334155;
  text-align: center;
}

.size-input:focus {
  outline: none;
  border-color: #3b82f6;
  background-color: #fff;
}

.apply-button {
  background-color: #3b82f6;
  color: white;
  border: none;
  font-weight: 500;
  justify-content: center;
  margin-top: 4px;
}

.apply-button:hover {
  background-color: #2563eb;
}

.apply-button .button-icon {
  color: white;
}

/* 标签页样式 */
.tab-headers {
  display: flex;
  margin-bottom: 16px;
  border-bottom: 1px solid #e2e8f0;
}

.tab-button {
  padding: 10px 15px;
  background: none;
  border: none;
  font-size: 14px;
  font-weight: 500;
  color: #64748b;
  cursor: pointer;
  position: relative;
  transition: all 0.2s;
  flex: 1;
  text-align: center;
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 6px;
}

.tab-button:hover {
  color: #3b82f6;
}

.tab-button.active {
  color: #3b82f6;
  font-weight: 600;
}

.tab-button.active::after {
  content: '';
  position: absolute;
  bottom: -1px;
  left: 0;
  right: 0;
  height: 2px;
  background-color: #3b82f6;
  border-radius: 2px 2px 0 0;
}

.tab-icon {
  fill: currentColor;
  transition: transform 0.2s ease;
}

.tab-button:hover .tab-icon {
  transform: scale(1.1);
}

.tab-button.active .tab-icon {
  fill: #3b82f6;
  transform: scale(1.2);
}

.tab-content {
  padding: 10px 0;
  animation: fadeIn 0.3s ease;
}

/* 描述文本样式 */
.description-text {
  font-size: 13px;
  color: #64748b;
  margin-bottom: 10px;
  line-height: 1.4;
  background-color: #f1f5f9;
  padding: 8px 10px;
  border-radius: 4px;
  border-left: 3px solid #cbd5e1;
}

/* 区域创建状态提示 */
.area-creation-hint {
  position: fixed;
  bottom: 20px;
  left: 50%;
  transform: translateX(-50%);
  background-color: rgba(51, 65, 85, 0.9);
  color: white;
  padding: 10px 16px;
  border-radius: 8px;
  font-size: 14px;
  z-index: 2000;
  box-shadow: 0 4px 12px rgba(0, 0, 0, 0.15);
  display: flex;
  align-items: center;
  gap: 8px;
}

.hint-icon {
  color: #60a5fa;
}

@keyframes fadeIn {
  from { opacity: 0; transform: translateY(-10px); }
  to { opacity: 1; transform: translateY(0); }
}
.group-label {
  position: absolute;
  top: -30px;
  right: 10px;
  padding: 4px 8px;
  border-radius: 4px;
  color: white;
  font-size: 12px;
  font-weight: bold;
  box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);
  z-index: 100;
}

/* 区域样式 */
.area {
  position: absolute;
  cursor: pointer;
  transition: all 0.2s ease;
  z-index: 5;
}

.area:hover {
  box-shadow: 0 0 0 2px #fff, 0 0 0 4px rgba(59, 130, 246, 0.5);
}

/* 区域菜单样式 */
.area-menu {
  width: 280px;
  box-shadow: 0 6px 24px rgba(0, 0, 0, 0.18);
  border: 1px solid rgba(59, 130, 246, 0.3);
  max-height: 80vh; /* 限制最大高度 */
  overflow-y: auto; /* 内容过多时允许滚动 */
}

/* 颜色选择器和删除确认内容样式 */
.area-menu .color-picker-content,
.area-menu .delete-confirm-content {
  position: relative; /* 确保内容不会溢出菜单 */
  max-height: 300px;
  overflow-y: auto;
}

.name-input {
  width: 100%;
  padding: 8px 10px;
  border: 1px solid #e2e8f0;
  border-radius: 4px;
  font-size: 14px;
  transition: all 0.2s;
}

.name-input:focus {
  border-color: #3b82f6;
  box-shadow: 0 0 0 2px rgba(59, 130, 246, 0.2);
  outline: none;
}

.color-preview {
  width: 24px;
  height: 24px;
  border-radius: 4px;
  border: 1px solid rgba(0, 0, 0, 0.1);
}
</style>
