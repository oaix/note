# Snippets

## numpy

### 获取list中唯一元素集合

```python
pc_map_index = np.fromfile(map_name, dtype=np.uint32)
pc_map_index_unique, pc_map_index_new = np.unique(pc_map_index, return_inverse = True) # (num, )
pc_map = self.pc.pc_data[pc_map_index_unique]

```

