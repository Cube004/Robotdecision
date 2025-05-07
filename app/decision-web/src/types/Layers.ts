import { ref } from 'vue';

export const layers = ref([
  { name: '节点图层', type: 'node', visible: true, locked: false, color: '#3B82F6' },
  { name: '连接线图层', type: 'edge', visible: true, locked: false, color: '#10B981' },
  { name: '编组图层', type: 'group', visible: true, locked: false, color: '#F59E0B' },
  { name: '航点图层', type: 'waypoint', visible: true, locked: true, color: '#3B82F6' },
  { name: '区域图层', type: 'area', visible: true, locked: true, color: '#3B82F6' },
  { name: '地图设置图层', type: 'mapPoint', visible: true, locked: true, color: '#3B82F6' },
]);