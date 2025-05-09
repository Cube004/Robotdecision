import { ref } from 'vue';

export const path_node = ref<number[]>([])

export const current_data = ref<{label: string, data: number}[]>([])

export const pose = ref<{x: number, y: number, z: number}[]>([])