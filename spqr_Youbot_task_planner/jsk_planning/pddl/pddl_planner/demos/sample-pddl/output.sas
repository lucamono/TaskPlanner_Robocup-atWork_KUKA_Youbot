begin_version
3
end_version
begin_metric
0
end_metric
7
begin_variable
var0
-1
2
Atom clear(a)
NegatedAtom clear(a)
end_variable
begin_variable
var1
-1
2
Atom clear(b)
NegatedAtom clear(b)
end_variable
begin_variable
var2
-1
2
Atom clear(c)
NegatedAtom clear(c)
end_variable
begin_variable
var3
-1
2
Atom handempty()
NegatedAtom handempty()
end_variable
begin_variable
var4
-1
4
Atom holding(a)
Atom on(a, b)
Atom on(a, c)
Atom ontable(a)
end_variable
begin_variable
var5
-1
4
Atom holding(b)
Atom on(b, a)
Atom on(b, c)
Atom ontable(b)
end_variable
begin_variable
var6
-1
4
Atom holding(c)
Atom on(c, a)
Atom on(c, b)
Atom ontable(c)
end_variable
7
begin_mutex_group
4
0 0
4 0
5 1
6 1
end_mutex_group
begin_mutex_group
4
1 0
5 0
4 1
6 2
end_mutex_group
begin_mutex_group
4
2 0
6 0
4 2
5 2
end_mutex_group
begin_mutex_group
4
3 0
4 0
5 0
6 0
end_mutex_group
begin_mutex_group
4
4 0
4 1
4 2
4 3
end_mutex_group
begin_mutex_group
4
5 0
5 1
5 2
5 3
end_mutex_group
begin_mutex_group
4
6 0
6 1
6 2
6 3
end_mutex_group
begin_state
1
0
0
0
3
3
1
end_state
begin_goal
2
4 1
5 2
end_goal
18
begin_operator
pickup a
0
3
0 0 0 1
0 3 0 1
0 4 3 0
0
end_operator
begin_operator
pickup b
0
3
0 1 0 1
0 3 0 1
0 5 3 0
0
end_operator
begin_operator
pickup c
0
3
0 2 0 1
0 3 0 1
0 6 3 0
0
end_operator
begin_operator
putdown a
0
3
0 0 -1 0
0 3 -1 0
0 4 0 3
0
end_operator
begin_operator
putdown b
0
3
0 1 -1 0
0 3 -1 0
0 5 0 3
0
end_operator
begin_operator
putdown c
0
3
0 2 -1 0
0 3 -1 0
0 6 0 3
0
end_operator
begin_operator
stack a b
0
4
0 0 -1 0
0 1 0 1
0 3 -1 0
0 4 0 1
0
end_operator
begin_operator
stack a c
0
4
0 0 -1 0
0 2 0 1
0 3 -1 0
0 4 0 2
0
end_operator
begin_operator
stack b a
0
4
0 0 0 1
0 1 -1 0
0 3 -1 0
0 5 0 1
0
end_operator
begin_operator
stack b c
0
4
0 1 -1 0
0 2 0 1
0 3 -1 0
0 5 0 2
0
end_operator
begin_operator
stack c a
0
4
0 0 0 1
0 2 -1 0
0 3 -1 0
0 6 0 1
0
end_operator
begin_operator
stack c b
0
4
0 1 0 1
0 2 -1 0
0 3 -1 0
0 6 0 2
0
end_operator
begin_operator
unstack a b
0
4
0 0 0 1
0 1 -1 0
0 3 0 1
0 4 1 0
0
end_operator
begin_operator
unstack a c
0
4
0 0 0 1
0 2 -1 0
0 3 0 1
0 4 2 0
0
end_operator
begin_operator
unstack b a
0
4
0 0 -1 0
0 1 0 1
0 3 0 1
0 5 1 0
0
end_operator
begin_operator
unstack b c
0
4
0 1 0 1
0 2 -1 0
0 3 0 1
0 5 2 0
0
end_operator
begin_operator
unstack c a
0
4
0 0 -1 0
0 2 0 1
0 3 0 1
0 6 1 0
0
end_operator
begin_operator
unstack c b
0
4
0 1 -1 0
0 2 0 1
0 3 0 1
0 6 2 0
0
end_operator
0
