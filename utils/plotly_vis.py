import numpy as np

import plotly.graph_objs as go
import plotly.io as pio
from scipy.spatial.transform import Rotation



from cv_bridge import CvBridge

from flask import Flask, render_template_string

from scipy.spatial.transform import Rotation as R

# source: https://github.com/YanjieZe/3D-Diffusion-Policy/blob/master/visualizer/visualizer/pointcloud.py

    
class Visualizer:
    def __init__(self):
        self.app = Flask(__name__)
        self.pointclouds = []
        
    def _generate_trace(self, pointcloud, color:tuple=None, size=3, opacity=0.7):
        x_coords = pointcloud[:, 0]
        y_coords = pointcloud[:, 1]
        z_coords = pointcloud[:, 2]

        if pointcloud.shape[1] == 3:
            if color is None:
                # design a colorful point cloud based on 3d coordinates
                # Normalize coordinates to range [0, 1]
                min_coords = pointcloud.min(axis=0)
                max_coords = pointcloud.max(axis=0)
                normalized_coords = (pointcloud - min_coords) / (max_coords - min_coords)
                try:
                    # Use normalized coordinates as RGB values
                    colors = ['rgb({},{},{})'.format(int(r*255), int(g*255), int(b*255)) for r, g, b in normalized_coords]
                except: # maybe meet NaN error
                    # use simple cyan color
                    colors = ['rgb(0,255,255)' for _ in range(len(x_coords))]
            else:    
                colors = ['rgb({},{},{})'.format(color[0], color[1], color[2]) for _ in range(len(x_coords))]
        else:
            colors = ['rgb({},{},{})'.format(int(r), int(g), int(b)) for r, g, b in pointcloud[:, 3:6]]

        return go.Scatter3d(
            x=x_coords,
            y=y_coords,
            z=z_coords,
            mode='markers',
            marker=dict(
                size=size,
                opacity=opacity,
                color=colors
            )
        )


    def colorize(self, pointcloud):
        if pointcloud.shape[1] == 3:

            # design a colorful point cloud based on 3d coordinates
            # Normalize coordinates to range [0, 1]
            min_coords = pointcloud.min(axis=0)
            max_coords = pointcloud.max(axis=0)
            normalized_coords = (pointcloud - min_coords) / (max_coords - min_coords)
            try:
                # Use normalized coordinates as RGB values
                colors = ['rgb({},{},{})'.format(int(r*255), int(g*255), int(b*255)) for r, g, b in normalized_coords]
            except: # maybe meet NaN error
                # use simple cyan color
                x_coords = pointcloud[:, 0]
                colors = ['rgb(0,255,255)' for _ in range(len(x_coords))]

        else:
            colors = ['rgb({},{},{})'.format(int(r), int(g), int(b)) for r, g, b in pointcloud[:, 3:6]]
        return colors
    

    def visualize_pointcloud(self, pointcloud, color:tuple=None):
        trace = self._generate_trace(pointcloud, color=color, size=6, opacity=1.0)
        layout = go.Layout(margin=dict(l=0, r=0, b=0, t=0))
        fig = go.Figure(data=[trace], layout=layout)
        
        fig.update_layout(
            
            scene=dict(
                # aspectmode='cube', 
                xaxis=dict(
                    showbackground=False,  
                    showgrid=True,       
                    showline=True,       
                    linecolor='grey',    
                    zerolinecolor='grey',  
                    zeroline=False,        
                    gridcolor='grey',     
                    
                ),
                yaxis=dict(
                    showbackground=False,
                    showgrid=True,
                    showline=True,
                    linecolor='grey',
                    zerolinecolor='grey',
                    zeroline=False,      
                    gridcolor='grey',    
                ),
                zaxis=dict(
                    showbackground=False,
                    showgrid=True,
                    showline=True,
                    linecolor='grey',
                    zerolinecolor='grey',
                    zeroline=False,       
                    gridcolor='grey',     
                ),
                bgcolor='white'  
            )
        )
        div = pio.to_html(fig, full_html=False)

        @self.app.route('/')
        def index():
            return render_template_string('''<div>{{ div|safe }}</div>''', div=div)
        
        self.app.run(debug=True, use_reloader=False)

    def visualize_pointcloud_and_save(self, pointcloud, color:tuple=None, save_path=None):
        trace = self._generate_trace(pointcloud, color=color, size=6, opacity=1.0)
        layout = go.Layout(margin=dict(l=0, r=0, b=0, t=0))
        fig = go.Figure(data=[trace], layout=layout)
        
        fig.update_layout(
            
            scene=dict(
                # aspectmode='cube', 
                xaxis=dict(
                    showbackground=False,  
                    showgrid=True,        
                    showline=True,        
                    linecolor='grey',    
                    zerolinecolor='grey',
                    zeroline=False,        
                    gridcolor='grey',     
                    
                ),
                yaxis=dict(
                    showbackground=False,
                    showgrid=True,
                    showline=True,
                    linecolor='grey',
                    zerolinecolor='grey',
                    zeroline=False,     
                    gridcolor='grey',    
                ),
                zaxis=dict(
                    showbackground=False,
                    showgrid=True,
                    showline=True,
                    linecolor='grey',
                    zerolinecolor='grey',
                    zeroline=False,      
                    gridcolor='grey',     
                ),
                bgcolor='white'  
            )
        )
        # save
        fig.write_image(save_path, width=800, height=600)
        

    def save_visualization_to_file(self, pointcloud, space_range, file_path=None, color:tuple=None):
        # visualize pointcloud and save as html

        x_min = space_range[0]
        x_max = space_range[1]
        y_min = space_range[2]
        y_max = space_range[3]
        z_min = space_range[4]
        z_max = space_range[5]


        vertices = [
            [x_min, y_min, z_min], [x_max, y_min, z_min],
            [x_max, y_max, z_min], [x_min, y_max, z_min],
            [x_min, y_min, z_max], [x_max, y_min, z_max],
            [x_max, y_max, z_max], [x_min, y_max, z_max]
        ]

        # Unzip the vertices into separate x, y, z lists
        x_cube, y_cube, z_cube = zip(*vertices)

        # Define the lines connecting the vertices of the cube
        edges = [
            [0, 1], [1, 2], [2, 3], [3, 0],  # Bottom face
            [4, 5], [5, 6], [6, 7], [7, 4],  # Top face
            [0, 4], [1, 5], [2, 6], [3, 7]   # Vertical edges
        ]

        # Create the lines for the cube
        cube_lines = []
        for edge in edges:
            cube_lines.append(
                go.Scatter3d(
                    x=[x_cube[edge[0]], x_cube[edge[1]]],
                    y=[y_cube[edge[0]], y_cube[edge[1]]],
                    z=[z_cube[edge[0]], z_cube[edge[1]]],
                    mode='lines',
                    line=dict(color='grey', width=1),  # Cube line color set to black
                    showlegend=False
                )
            )


        trace = self._generate_trace(pointcloud, color=color)
        # layout = go.Layout(margin=dict(l=0, r=0, b=0, t=0))

        layout = go.Layout(
        margin=dict(l=0, r=0, b=0, t=0),)
        

    
        # fig = go.Figure(data=[trace] + cube_lines, layout=layout)

        fig = go.Figure(data=[trace] , layout=layout)

        delta = 0.1
        fig.update_layout(
            scene=dict(
                xaxis=dict(range=[x_min-delta, x_max+delta]),  # Set x-axis limits
                yaxis=dict(range=[y_min-delta, y_max+delta]),  # Set y-axis limits
                zaxis=dict(range=[z_min, z_max+delta])),   # Set z-axis limits
                scene_aspectmode='manual',
                scene_aspectratio=dict(x=x_max-x_min, y=y_max-y_min, z=z_max-z_min))

        

        fig.update_layout(
            
            scene=dict(
                # aspectmode='cube', 
                xaxis=dict(
                    showbackground=False,  
                    showgrid=True,       
                    showline=True,       
                    linecolor='grey',     
                    zerolinecolor='grey', 
                    zeroline=False,        
                    gridcolor='grey', 
                    linewidth=0.5,     
                    showticklabels=False,   
                    title='',           
                    dtick= (x_max-x_min)/2 ,             

    
                    
                ),

                yaxis=dict(
                    showbackground=False,
                    showgrid=True,
                    showline=True,
                    linecolor='grey',
                    zerolinecolor='grey',
                    zeroline=False,     
                    gridcolor='grey',  
                    linewidth=0.5,     
                    showticklabels=False,   
                    title='',           
                    dtick= (y_max-y_min)/2 ,    
                ),
                
                zaxis=dict(
                    showbackground=False,
                    showgrid=True,
                    showline=True,
                    linecolor='grey',
                    zerolinecolor='grey',
                    zeroline=False,       
                    gridcolor='grey',    
                    linewidth=0.5,     
                    showticklabels=False,   
                    title='',           
                    dtick= (z_max-z_min)/2 ,    
                ),
                bgcolor='white' 
            )
        )
   
   
        # set background color as white
        fig.update_layout(template='none')


        # camera_pose = {
        #     'eye': dict(x=0.6, y=0, z=-0.4),   # Position of the camera
        #     'up': dict(x=0, y=0, z=-1),     # Up direction for the camera
        #     'center': dict(x=0.2, y=0, z=0.5)   # Point the camera is looking at
        # }

        # fig.update_layout(
        #     scene=dict(
        #         camera=dict(
        #             eye=camera_pose.get('eye'),  # Default position
        #             up=camera_pose.get('up'),  # Default up direction
        #             center=camera_pose.get('center'),   # Default center point
        #         )
        #     )
        # )
    
    
        if file_path is None:
            return fig
        else:
                
            fig_html = pio.to_html(fig, full_html=True)

            with open(file_path, 'w') as file:
                file.write(fig_html)
            print(f"Visualization saved to {file_path}")


    def save_pointclouds_interactive(self, pointclouds, space_range, file_path):
        """
        Create an interactive visualization for a list of point clouds.

        Parameters:
        - pointclouds: A list of point clouds (each as an Nx3 array-like).
        - file_path: The path to save the HTML file.
        """
        # Create a list to hold each point cloud's figure
        figures = []
        
        # Generate a figure for each point cloud
        for pc in pointclouds:

            fig = self.save_visualization_to_file(pc, space_range)


            figures.append(fig)

        # Create HTML content with JavaScript for interactivity
       
        # Create HTML content with JavaScript for interactivity
        html_content = '''
        <html>
        <head>
            <title>Point Cloud Visualization</title>
            <script src="https://cdn.plot.ly/plotly-latest.min.js"></script>
            <style>
                body { display: flex; flex-direction: column; align-items: center; height: 100vh; }
                #plot { width: 100%; height: 80%; }  /* Give more space to the plot */
                .slider { margin: 10px; width: 50%; }  /* Smaller width for the slider */
                #slider-container { position: absolute; bottom: 20px; width: 100%; }  /* Position slider at the bottom */
            </style>
        </head>
        <body>
            <div id="plot"></div>
            <div id="slider-container">
                <input type="range" id="slider" class="slider" min="0" max="''' + str(len(pointclouds) - 1) + '''" value="0" step="1">
            </div>
            <script>
                const figures = {}
                ''' + ''.join(f'figures["fig{idx}"] = {fig.to_json()};\n' for idx, fig in enumerate(figures)) + '''
                const plotDiv = document.getElementById('plot');
                const slider = document.getElementById('slider');

                function renderFigure(index) {
                    Plotly.newPlot(plotDiv, figures['fig' + index]);
                }

                slider.oninput = function() {
                    renderFigure(this.value);
                };

                // Initial render
                renderFigure(0);
            </script>
        </body>
        </html>
        '''



        # Write the HTML content to a file
        with open(file_path, 'w') as file:
            file.write(html_content)

        print(f"Interactive point cloud visualization saved to {file_path}")
