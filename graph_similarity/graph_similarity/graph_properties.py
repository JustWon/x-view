class _GraphProperties:
    """Global graph property class containing keys to access node attributes.
    Note: this class should not be instantiated by the user, as an instance of this class is provided as
    'GraphProperties' (see below).
    """
    def __init__(self):
        self.node_unique_identifier_key = None
        self.graph_membership_key = None
        self.node_semantic_label_id_key = None
        self.node_semantic_label_key = None
        self.node_drawing_position_key = None


# Graph properties accessed by the entire program.
GraphProperties = _GraphProperties
GraphProperties.node_unique_identifier_key = "unique_id"
GraphProperties.graph_membership_key = "graph_membership_id"
GraphProperties.node_semantic_label_id_key = "semantic_label_id"
GraphProperties.node_semantic_label_key = "semantic_label"
GraphProperties.node_drawing_position_key = "drawing_position"
