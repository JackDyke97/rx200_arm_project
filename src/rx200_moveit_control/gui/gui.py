import rclpy
import sys
import threading
from rclpy.node import Node
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QSlider, QLabel, QDoubleSpinBox, QScrollArea
from PyQt5.QtCore import Qt
from std_msgs.msg import Float32


class GUIThing(Node):
    def __init__(self):
        super().__init__("Graphic_interface")
        self.pub = self.create_publisher(Float32, '/filter_threshold', 10)

        #parameters for crop box based on interbotix ui
        self.pub_x_min = self.create_publisher(Float32, '/cropbox/x_min', 10)
        self.pub_x_max = self.create_publisher(Float32, '/cropbox/x_max', 10)
        self.pub_y_min = self.create_publisher(Float32, '/cropbox/y_min', 10)
        self.pub_y_max = self.create_publisher(Float32, '/cropbox/y_max', 10)
        self.pub_z_min = self.create_publisher(Float32, '/cropbox/z_min', 10)
        self.pub_z_max = self.create_publisher(Float32, '/cropbox/z_max', 10)

        #parameter for leaf size
        self.pub_leaf_size = self.create_publisher(Float32, '/voxelgrid/leaf_size', 10)
        
        #parameters for segmentation filter
        self.pub_dist_threshold = self.create_publisher(Float32, '/seg/dist_thresh', 10)
        self.pub_max_iterations = self.create_publisher(Float32, '/seg/max_iter', 10)

        #parameters for radius outlier removal
        self.pub_min_neighbours = self.create_publisher(Float32, '/ror/min_neighbors', 10)
        self.pub_radius_search = self.create_publisher(Float32, '/ror/radius', 10)

        #parameters for cluster filter
        self.pub_min_size = self.create_publisher(Float32, '/cluster/min_size', 10)
        self.pub_max_size = self.create_publisher(Float32, '/cluster/max_size', 10)
        self.pub_tolerance = self.create_publisher(Float32, '/cluster/tolerance', 10)



class CropBoxFilter(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        header = QLabel("Crop Box Filter")
        
        layout = QVBoxLayout()
        layout.addWidget(header)
        layout.addWidget(Sliders('X min [m]', -0.5, 0.5, 2, ros_node.pub_x_min))
        layout.addWidget(Sliders('X max [m]', -0.5, 0.5, 2, ros_node.pub_x_max))
        layout.addWidget(Sliders('Y min [m]', -0.5, 0.5, 2, ros_node.pub_y_min))
        layout.addWidget(Sliders('Y max [m]', -0.5, 0.5, 2, ros_node.pub_y_max))
        layout.addWidget(Sliders('Z min [m]', -0.5, 0.5, 2, ros_node.pub_z_min))
        layout.addWidget(Sliders('Z max [m]', -0.5, 0.5, 2, ros_node.pub_z_max))

        # self.slider.setRange(0, 100)
        # self.slider.setValue(0)
        # self.slider.valueChanged.connect(self.slider_move)

        self.setLayout(layout)

class VoxelGridFilter(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        header = QLabel("Voxel Grid Filter")
        layout = QVBoxLayout()
        layout.addWidget(header)

        layout.addWidget(Sliders('Leaf Size [m]', 0.001, 0.01, 3, ros_node.pub_leaf_size))

        self.setLayout(layout)

class SegmentationFilter(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        header = QLabel("Segmentation Filter")
        layout = QVBoxLayout()
        layout.addWidget(header)

        layout.addWidget(Sliders("Dist. Threshold [m]", 0.001, 0.05, 3, ros_node.pub_dist_threshold))
        layout.addWidget(Sliders('Max Iteration ', 25, 1000, 0, ros_node.pub_max_iterations))

        self.setLayout(layout)

class RadiusOutlier(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        header = QLabel("Radius Outlier Removal Filter")
        layout = QVBoxLayout()
        layout.addWidget(header)

        layout.addWidget(Sliders('Min Neighbours', 1, 20, 0, ros_node.pub_min_neighbours))
        layout.addWidget(Sliders('Radius Search [m]', 0.005, 0.05, 3, ros_node.pub_radius_search))

        self.setLayout(layout)

class ClusterFilter(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        header = QLabel("Cluster Filter")
        layout = QVBoxLayout()
        layout.addWidget(header)

        layout.addWidget(Sliders('Min Size', 25, 1000, 0, ros_node.pub_min_size))
        layout.addWidget(Sliders('Max Size', 25, 1000, 0, ros_node.pub_max_size))
        layout.addWidget(Sliders('Tolerance [m]', 0.01, 0.1, 2, ros_node.pub_tolerance))

        self.setLayout(layout)




# class SimpleFilters(QWidget):
#     def __init__(self, ros_node):
#         super().__init__()
#         self.ros_node = ros_node
#         self.setWindowTitle("Simple gui")

#         layout = QVBoxLayout()
#         self.label = QLabel("Crop Box Filter: 50")
#         layout.addWidget(self.label)

#         self.slider = QSlider(Qt.Horizontal)
#         self.slider.setRange(0, 100)
#         self.slider.setValue(50)
#         self.slider.valueChanged.connect(self.slider_move)
#         layout.addWidget(self.slider)

#         self.setLayout(layout)
    
  
#for the sliders
class Sliders(QWidget):
    def __init__(self, label_text, min_val, max_val, precision, ros_publisher):
        super().__init__()
        self.min_val = min_val
        self.max_val = max_val
        self.precision = precision
        self.pub = ros_publisher
        self.label_text = label_text

        layout = QVBoxLayout()
        self.label = QLabel(f"{label_text}: 0.0")
        layout.addWidget(self.label)

        self.spin = QDoubleSpinBox()
        self.spin.setDecimals(precision)
        self.spin.setRange(min_val, max_val)
        self.spin.setSingleStep(10**(-precision))
        layout.addWidget(self.spin)

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(0, int((max_val - min_val) * (10**precision)))
        self.slider.setValue(0)
        self.slider.valueChanged.connect(self.slider_move)
        layout.addWidget(self.slider)

        self.setLayout(layout)

        self.spin.setValue((min_val + max_val) / 2)

        self.spin.valueChanged.connect(self.spin_changed)
        self.slider.valueChanged.connect(self.slider_move)
    
    def spin_changed(self, value):
        slider_pos = int((value - self.min_val) * (10**self.precision))

        self.slider.blockSignals(True)
        self.slider.setValue(slider_pos)
        self.slider.blockSignals(False)
        self.label.setText(f'{self.label_text}: {value:.{self.precision}f}')

        msg = Float32()
        msg.data = float(value)
        self.pub.publish(msg)

    def slider_move(self, val):
        position = self.min_val + (val / (10**self.precision))

        self.spin.blockSignals(True)
        self.spin.setValue(position)
        self.spin.blockSignals(False)
        self.label.setText(f"{self.label_text}: {position:.{self.precision}f}")    

        msg = Float32()
        msg.data = float(position)
        self.pub.publish(msg)


class OneWindow(QWidget):
    def __init__(self, ros_node):
        super().__init__()

        layout = QVBoxLayout()
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        container = QWidget()
        conatainer_layout = QVBoxLayout(container)

        conatainer_layout.addWidget(CropBoxFilter(ros_node))
        conatainer_layout.addWidget(VoxelGridFilter(ros_node))
        conatainer_layout.addWidget(SegmentationFilter(ros_node))
        conatainer_layout.addWidget(RadiusOutlier(ros_node))
        conatainer_layout.addWidget(ClusterFilter(ros_node))

        scroll.setWidget(container)
        layout.addWidget(scroll)
        self.setLayout(layout)

def main():
    rclpy.init()
    ros_node = GUIThing()

    app = QApplication(sys.argv)
    # cropbox = CropBoxFilter(ros_node)
    # voxelgrid = VoxelGridFilter(ros_node)
    # cropbox.show()
    # voxelgrid.show()
    uiWindow = OneWindow(ros_node)
    uiWindow.show()

    ros_spin_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    ros_spin_thread.start()

    sys.exit(app.exec_())
    ros_node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()

