from stl import mesh
import os

# print(os.getcwd())
# Load the STL file
your_mesh = mesh.Mesh.from_file(fr'/home/bryan/Downloads/Round trainer eduted v1.2/meshes/base_link.STL')

# Access all the coordinates
all_coordinates = your_mesh.vectors  # Each face is represented by three vertices
print("Coordinates of the solid:")
print(all_coordinates)

# Flatten the coordinates to get a list of unique vertices
unique_coordinates = set(tuple(vertex) for triangle in all_coordinates for vertex in triangle)
print("Unique vertices:")
print(unique_coordinates)
xl = []
yl = []
zl = []
for x,y,z in unique_coordinates:
    xl.append(x)
    yl.append(y)
    zl.append(z)
print(1)
print(max(zl))
