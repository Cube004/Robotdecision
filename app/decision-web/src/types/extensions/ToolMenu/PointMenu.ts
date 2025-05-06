import { ref, watch, watchEffect } from 'vue';
import { Point } from '@/types/Point';
import { updatePointMenuPosition } from './MapPage';
import { showPointMenu } from './MapPage';
import { points } from '@/types/Manger';
import { MapSettingsPoints, mapWidth, mapHeight } from '@/types/Manger';

// 菜单状态
export const mapImage = ref<HTMLImageElement | null>(null);
export const showColorPicker = ref(false);
export const showCustomColor = ref(false);
export const showTextProperties = ref(false);
export const showDeleteConfirm = ref(false);
export const selectedPoint = ref<Point | null>(null);


// 菜单属性
export const selectedColor = ref('#4285f4');
export const customColor = ref('#4285f4');
export const colorOptions = ref(['#000000', '#5f6368', '#bdc1c6', '#4285f4', '#8e24aa', '#4052b5', '#137333', '#b06000', '#ea4335']);
export const textColorOptions = ref(['#000000', '#5f6368', '#4285f4', '#137333', '#ea4335']);
export const pointText = ref('');
export const fontSize = ref(14);
export const selectedTextColor = ref('#000000');

// 重置菜单状态
export function resetMenuState(){
  showColorPicker.value = false;
  showCustomColor.value = false;
  showTextProperties.value = false;
  showDeleteConfirm.value = false;
  selectedPoint.value = null;
};

// 创建新航点
export function createNewPoint(x: number, y: number) {
  const scale = getScale();
  if (!scale) return;
  const { scaleX, scaleY } = scale;
  const newPoint = new Point(
    points.value.length,
    { x, y },
    { x, y },
    selectedColor.value,
    pointText.value,
    fontSize.value,
    selectedTextColor.value,
    { x: scaleX, y: scaleY }
  );
  points.value.push(newPoint);
  selectedPoint.value = newPoint;
  showPointMenu.value = true;
};

// 选择附近的点
export function selectPointNearby( x: number, y: number) {
  const findNearbyPoint = (x: number, y: number) => {
    const threshold = 20; // 判定距离阈值

    for (const point of points.value) {
      const distance = Math.sqrt(Math.pow(point.position.x - x, 2) + Math.pow(point.position.y - y, 2));
      if (distance <= threshold) {
        return point;
      }
    }
    return null;
  };

  const nearbyPoint = findNearbyPoint(x, y);

  if (nearbyPoint) {
    selectedPoint.value = nearbyPoint;
    showPointMenu.value = true;
    console.log("showPointMenu",showPointMenu.value);

    pointText.value = nearbyPoint.text.value;
    fontSize.value = nearbyPoint.fontSize.value;
    selectedColor.value = nearbyPoint.color.value;
    selectedTextColor.value = nearbyPoint.textColor.value;
    updatePointMenuPosition(nearbyPoint.position.x, nearbyPoint.position.y);
  } else {
    // 如果没有点击到点，隐藏菜单
    showPointMenu.value = false;
    resetMenuState();
  }
};

// 通过ID选择点
export function selectPointById(id: number) {
  const point = points.value.find(p => p.id.value === id);
  if (point) {
    selectedPoint.value = point;
    showPointMenu.value = true;
    pointText.value = point.text.value;
    fontSize.value = point.fontSize.value;
    selectedColor.value = point.color.value;
    selectedTextColor.value = point.textColor.value;
    updatePointMenuPosition(point.position.x, point.position.y);
    showPointMenu.value = true;
  }
};


const resizeObserver = new ResizeObserver(entries => {
  for (const entry of entries) {
      const { width, height } = entry.contentRect;
      if (width && height) {
        const scale = getScale();
        if (!scale) return;
        const { scaleX, scaleY } = scale;
      [points, MapSettingsPoints].forEach(points => {
        points.value.forEach(point => {
          const x = point.position.x / scaleX * point.scale.x;
          const y = point.position.y / scaleY * point.scale.y;
          point.position.x = x;
          point.position.y = y;
          point.scale.x = scaleX;
          point.scale.y = scaleY;
        })
      })
    }
  }
  calculateWaypoint();
});

const stop = watchEffect(() => {
  if (mapImage.value) {
    resizeObserver.observe(mapImage.value);
    stop();
  }
})

export function getScale() {
  const displayWidth = mapImage.value?.width;
  const displayHeight = mapImage.value?.height;

  const naturalWidth = mapImage.value?.naturalWidth;
  const naturalHeight = mapImage.value?.naturalHeight;

  if (naturalWidth === undefined || naturalHeight === undefined || displayWidth === undefined || displayHeight === undefined) {
    console.log("getScale error");
    return;
  }
  const scaleX = naturalWidth / displayWidth;
  const scaleY = naturalHeight / displayHeight;

  return { scaleX: scaleX as number, scaleY: scaleY as number };
}

export function calculateWaypoint() {
  if (MapSettingsPoints.value.length === 3){
    points.value.forEach(point => {
      const width = MapSettingsPoints.value[2].position.x - MapSettingsPoints.value[1].position.x;
      const height = MapSettingsPoints.value[2].position.y - MapSettingsPoints.value[1].position.y;
      const x = point.position.x - MapSettingsPoints.value[0].position.x;
      const y = point.position.y - MapSettingsPoints.value[0].position.y;
      const waypoint = {
        x: x / width * mapWidth.value,
        y: y / height * mapHeight.value
      }
        point.waypoint = waypoint;
    });
  }
}
// // // // // //
// 航点编辑菜单 //
// // // // // //


// 切换颜色选择器显示状态
export function toggleColorPicker() {
  showColorPicker.value = !showColorPicker.value;
  showTextProperties.value = false;
  showDeleteConfirm.value = false;
};

// 切换自定义颜色显示状态
export function toggleCustomColor() {
  showCustomColor.value = !showCustomColor.value;
};

// 更新选择颜色
export function selectColor(color: string) {
  selectedColor.value = color;
  if (selectedPoint.value) {
    const point = points.value.find(p => p.id === selectedPoint.value!.id);
    if (point) {
      point.color.value = color;
    }
  }
};

// 添加自定义颜色
export function addCustomColor() {
  if (!colorOptions.value.includes(customColor.value)) {
    colorOptions.value.push(customColor.value);
  }
  selectColor(customColor.value);
  showCustomColor.value = false;
};

// 切换文本属性显示状态
export function toggleTextProperties() {
  showTextProperties.value = !showTextProperties.value;
  showColorPicker.value = false;
  showDeleteConfirm.value = false;
};

// 选择文本颜色
export function selectTextColor(color: string) {
  selectedTextColor.value = color;

  // 更新当前选中点的文本颜色
  if (selectedPoint.value) {
    const point = points.value.find(p => p.id === selectedPoint.value!.id);
    if (point) {
      point.textColor.value = color;
    }
  }
};

// 切换删除确认显示状态
export function toggleDeleteConfirm() {
  showDeleteConfirm.value = !showDeleteConfirm.value;
  showColorPicker.value = false;
  showTextProperties.value = false;
};

// 删除点
export function deletePoint() {
  if (selectedPoint.value) {
    points.value = points.value.filter(p => p.id !== selectedPoint.value!.id);
    selectedPoint.value = null;
    showPointMenu.value = false;
    resetMenuState();
  }
};

// 监听文本变化，更新点的文本内容
watch(pointText, (newText) => {
  if (selectedPoint.value) {
    const point = points.value.find(p => p.id === selectedPoint.value!.id);
    if (point) {
      point.text.value = newText;
    }
  }
});

// 监听字体大小变化，更新点的字体大小
watch(fontSize, (newSize) => {
  if (selectedPoint.value) {
    const point = points.value.find(p => p.id === selectedPoint.value!.id);
    if (point) {
      point.fontSize.value = newSize;
    }
  }
});
