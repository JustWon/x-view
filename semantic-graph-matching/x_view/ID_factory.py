class IdFactory(object):
    """IdFactory class used to generate unique ids for the graph nodes. This is necessary as
    when merging two graphs together, even if they have the same node we don't want the nodes to collapse together
    """
    __instance = None
    __current_id = 0

    def __new__(cls, *args, **kwargs):
        if not cls.__instance:
            cls.__instance = super(IdFactory, cls).__new__(cls, *args, **kwargs)
        return cls.__instance

    def next_id(self):
        """Generates a unique id each time it is called
        :return: a unique id (sequentially increasing) whenever this function is called
        """
        id = self.__current_id
        self.__current_id += 1
        return id


def next_id():
    return IdFactory().next_id()