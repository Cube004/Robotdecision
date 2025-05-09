import { ref } from 'vue';

export const path_node = ref<number[]>([])

export const current_data = ref<{label: string, data: number}[]>([])

export const pose = ref<{x: number, y: number, z: number}>()
pose.value = {x: 0, y: 0, z: 0}
export const push_data = ref<{label: string, data: number}[]>([])

export function init_push_data() {
  fetch('/rules/condition_rules.json')
  .then(response => response.json())
  .then(data => {
    if (Array.isArray(data.rules)) {
      data.rules.forEach((item: { type: string; description: string; data: number }) => {
        push_data.value.push({ label: item.type, data: item.data as number });
      });
    }
    console.log('成功加载数据类型:', push_data.value.length);
  })
  .catch(error => {
    console.error('加载文件时出错:', error);
  });
}

init_push_data()