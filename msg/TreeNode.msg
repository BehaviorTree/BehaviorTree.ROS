uint16         uid
uint16[]       children_uid
int8           type
NodeStatus     status
string         instance_name
string         registration_name
NodeParameter  params


int8 UNDEFINED = 0
int8 ACTION = 1
int8 CONDITION = 2
int8 CONTROL = 3
int8 DECORATOR = 4
int8 SUBTREE = 5
