Hydrographic Printing Simulator

Hydrographic Printing, also called Hydrographics, is a viable method for coloring objects created with 3D printers. However, executing the hydrographic technique leads to a complex interaction between a thin film and a 3D printed object,  in which the image in the film must adhere to the object. To handle the difficulty of predicting the final result of hydrographic printing, we propose a 3D computational simulation that uses Position-Based Dynamics, a popular technique for simulating deformable bodies and widely used in physics engines. We take advantage of this technique running in parallel a GPU-based simulation with suitable performance. We simulate the film behavior and its interaction with the printed object, as an interaction between a soft-body colliding with a rigid one. To evaluate the achieved performance consistency, we varied the number of vertices and voxels in the bodies involved and observed that the simulation kept running in real-time. We also execute the hydrographic technique in different printed models and compare these results with the simulated models.
