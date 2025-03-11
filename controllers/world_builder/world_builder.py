from controller import Supervisor

class WorldBuilder(Supervisor):
    def __init__(self):
        super().__init__()
        
    def build_roads(self):
        root = self.getRoot()
        children_field = root.getField('children')

        # Define road segment as a function for reusability
        def create_straight_road(x, y, rotation):
            return f"""
            StraightRoadSegment {{
              translation {x} {y} 0.1
              rotation 0 0 1 {rotation}
              rightBorder FALSE
              leftBorder FALSE
              length 60
              width 10
              numberOfLanes 1
            }}
            """

        # Define intersection
        def create_intersection(x, y):
            return f"""
            RoadIntersection {{
              translation {x} {y} 0
            }}
            """

        # Add horizontal roads
        children_field.importMFNodeFromString(-1, create_straight_road(0, 20, 0))
        children_field.importMFNodeFromString(-1, create_straight_road(0, -20, 0))

        # Add vertical roads
        children_field.importMFNodeFromString(-1, create_straight_road(-20, 0, 1.57))  # 90-degree rotation
        children_field.importMFNodeFromString(-1, create_straight_road(20, 0, 1.57))


if __name__ == '__main__':
    builder = WorldBuilder()
    builder.build_roads()
