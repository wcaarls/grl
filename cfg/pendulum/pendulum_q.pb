
?
inputPlaceholder*
dtype0*
shape:���������
L
h1/random_uniform/shapeConst*
valueB"   �  *
dtype0
B
h1/random_uniform/minConst*
valueB
 *�3��*
dtype0
B
h1/random_uniform/maxConst*
valueB
 *�3�=*
dtype0
~
h1/random_uniform/RandomUniformRandomUniformh1/random_uniform/shape*
seed2߫�*
seed���)*
T0*
dtype0
S
h1/random_uniform/subSubh1/random_uniform/maxh1/random_uniform/min*
T0
]
h1/random_uniform/mulMulh1/random_uniform/RandomUniformh1/random_uniform/sub*
T0
O
h1/random_uniformAddh1/random_uniform/mulh1/random_uniform/min*
T0
^
	h1/kernel
VariableV2*
dtype0*
	container *
shape:	�*
shared_name 
�
h1/kernel/AssignAssign	h1/kernelh1/random_uniform*
use_locking(*
T0*
_class
loc:@h1/kernel*
validate_shape(
L
h1/kernel/readIdentity	h1/kernel*
T0*
_class
loc:@h1/kernel
:
h1/ConstConst*
valueB�*    *
dtype0
X
h1/bias
VariableV2*
dtype0*
	container *
shape:�*
shared_name 
y
h1/bias/AssignAssignh1/biash1/Const*
_class
loc:@h1/bias*
validate_shape(*
use_locking(*
T0
F
h1/bias/readIdentityh1/bias*
T0*
_class
loc:@h1/bias
Y
	h1/MatMulMatMulinputh1/kernel/read*
transpose_a( *
transpose_b( *
T0
N

h1/BiasAddBiasAdd	h1/MatMulh1/bias/read*
T0*
data_formatNHWC
$
h1/ReluRelu
h1/BiasAdd*
T0
L
h2/random_uniform/shapeConst*
valueB"�  ,  *
dtype0
B
h2/random_uniform/minConst*
valueB
 *����*
dtype0
B
h2/random_uniform/maxConst*
valueB
 *���=*
dtype0
~
h2/random_uniform/RandomUniformRandomUniformh2/random_uniform/shape*
seed2�ž*
seed���)*
T0*
dtype0
S
h2/random_uniform/subSubh2/random_uniform/maxh2/random_uniform/min*
T0
]
h2/random_uniform/mulMulh2/random_uniform/RandomUniformh2/random_uniform/sub*
T0
O
h2/random_uniformAddh2/random_uniform/mulh2/random_uniform/min*
T0
_
	h2/kernel
VariableV2*
shared_name *
dtype0*
	container *
shape:
��
�
h2/kernel/AssignAssign	h2/kernelh2/random_uniform*
validate_shape(*
use_locking(*
T0*
_class
loc:@h2/kernel
L
h2/kernel/readIdentity	h2/kernel*
T0*
_class
loc:@h2/kernel
:
h2/ConstConst*
valueB�*    *
dtype0
X
h2/bias
VariableV2*
shape:�*
shared_name *
dtype0*
	container 
y
h2/bias/AssignAssignh2/biash2/Const*
_class
loc:@h2/bias*
validate_shape(*
use_locking(*
T0
F
h2/bias/readIdentityh2/bias*
T0*
_class
loc:@h2/bias
[
	h2/MatMulMatMulh1/Reluh2/kernel/read*
transpose_a( *
transpose_b( *
T0
N

h2/BiasAddBiasAdd	h2/MatMulh2/bias/read*
T0*
data_formatNHWC
$
h2/ReluRelu
h2/BiasAdd*
T0
P
output/random_uniform/shapeConst*
valueB",     *
dtype0
F
output/random_uniform/minConst*
valueB
 * ��*
dtype0
F
output/random_uniform/maxConst*
valueB
 * �>*
dtype0
�
#output/random_uniform/RandomUniformRandomUniformoutput/random_uniform/shape*
dtype0*
seed2���*
seed���)*
T0
_
output/random_uniform/subSuboutput/random_uniform/maxoutput/random_uniform/min*
T0
i
output/random_uniform/mulMul#output/random_uniform/RandomUniformoutput/random_uniform/sub*
T0
[
output/random_uniformAddoutput/random_uniform/muloutput/random_uniform/min*
T0
b
output/kernel
VariableV2*
shared_name *
dtype0*
	container *
shape:	�
�
output/kernel/AssignAssignoutput/kerneloutput/random_uniform*
validate_shape(*
use_locking(*
T0* 
_class
loc:@output/kernel
X
output/kernel/readIdentityoutput/kernel*
T0* 
_class
loc:@output/kernel
=
output/ConstConst*
valueB*    *
dtype0
[
output/bias
VariableV2*
shared_name *
dtype0*
	container *
shape:
�
output/bias/AssignAssignoutput/biasoutput/Const*
validate_shape(*
use_locking(*
T0*
_class
loc:@output/bias
R
output/bias/readIdentityoutput/bias*
T0*
_class
loc:@output/bias
c
output/MatMulMatMulh2/Reluoutput/kernel/read*
transpose_a( *
transpose_b( *
T0
Z
output/BiasAddBiasAddoutput/MatMuloutput/bias/read*
T0*
data_formatNHWC

inputsNoOp^input
 
outputsNoOp^output/BiasAdd
@
targetPlaceholder*
dtype0*
shape:���������
Z
$mean_squared_error/SquaredDifferenceSquaredDifferenceoutput/BiasAddtarget*
T0
\
/mean_squared_error/assert_broadcastable/weightsConst*
valueB
 *  �?*
dtype0
^
5mean_squared_error/assert_broadcastable/weights/shapeConst*
valueB *
dtype0
^
4mean_squared_error/assert_broadcastable/weights/rankConst*
value	B : *
dtype0
|
4mean_squared_error/assert_broadcastable/values/shapeShape$mean_squared_error/SquaredDifference*
T0*
out_type0
]
3mean_squared_error/assert_broadcastable/values/rankConst*
value	B :*
dtype0
K
Cmean_squared_error/assert_broadcastable/static_scalar_check_successNoOp
�
mean_squared_error/ToFloat/xConstD^mean_squared_error/assert_broadcastable/static_scalar_check_success*
valueB
 *  �?*
dtype0
j
mean_squared_error/MulMul$mean_squared_error/SquaredDifferencemean_squared_error/ToFloat/x*
T0
�
mean_squared_error/ConstConstD^mean_squared_error/assert_broadcastable/static_scalar_check_success*
valueB"       *
dtype0
u
mean_squared_error/SumSummean_squared_error/Mulmean_squared_error/Const*

Tidx0*
	keep_dims( *
T0
�
&mean_squared_error/num_present/Equal/yConstD^mean_squared_error/assert_broadcastable/static_scalar_check_success*
valueB
 *    *
dtype0
|
$mean_squared_error/num_present/EqualEqualmean_squared_error/ToFloat/x&mean_squared_error/num_present/Equal/y*
T0
�
)mean_squared_error/num_present/zeros_likeConstD^mean_squared_error/assert_broadcastable/static_scalar_check_success*
valueB
 *    *
dtype0
�
.mean_squared_error/num_present/ones_like/ShapeConstD^mean_squared_error/assert_broadcastable/static_scalar_check_success*
valueB *
dtype0
�
.mean_squared_error/num_present/ones_like/ConstConstD^mean_squared_error/assert_broadcastable/static_scalar_check_success*
valueB
 *  �?*
dtype0
�
(mean_squared_error/num_present/ones_likeFill.mean_squared_error/num_present/ones_like/Shape.mean_squared_error/num_present/ones_like/Const*
T0*

index_type0
�
%mean_squared_error/num_present/SelectSelect$mean_squared_error/num_present/Equal)mean_squared_error/num_present/zeros_like(mean_squared_error/num_present/ones_like*
T0
�
Smean_squared_error/num_present/broadcast_weights/assert_broadcastable/weights/shapeConstD^mean_squared_error/assert_broadcastable/static_scalar_check_success*
dtype0*
valueB 
�
Rmean_squared_error/num_present/broadcast_weights/assert_broadcastable/weights/rankConstD^mean_squared_error/assert_broadcastable/static_scalar_check_success*
value	B : *
dtype0
�
Rmean_squared_error/num_present/broadcast_weights/assert_broadcastable/values/shapeShape$mean_squared_error/SquaredDifferenceD^mean_squared_error/assert_broadcastable/static_scalar_check_success*
out_type0*
T0
�
Qmean_squared_error/num_present/broadcast_weights/assert_broadcastable/values/rankConstD^mean_squared_error/assert_broadcastable/static_scalar_check_success*
dtype0*
value	B :
�
amean_squared_error/num_present/broadcast_weights/assert_broadcastable/static_scalar_check_successNoOpD^mean_squared_error/assert_broadcastable/static_scalar_check_success
�
@mean_squared_error/num_present/broadcast_weights/ones_like/ShapeShape$mean_squared_error/SquaredDifferenceD^mean_squared_error/assert_broadcastable/static_scalar_check_successb^mean_squared_error/num_present/broadcast_weights/assert_broadcastable/static_scalar_check_success*
T0*
out_type0
�
@mean_squared_error/num_present/broadcast_weights/ones_like/ConstConstD^mean_squared_error/assert_broadcastable/static_scalar_check_successb^mean_squared_error/num_present/broadcast_weights/assert_broadcastable/static_scalar_check_success*
valueB
 *  �?*
dtype0
�
:mean_squared_error/num_present/broadcast_weights/ones_likeFill@mean_squared_error/num_present/broadcast_weights/ones_like/Shape@mean_squared_error/num_present/broadcast_weights/ones_like/Const*
T0*

index_type0
�
0mean_squared_error/num_present/broadcast_weightsMul%mean_squared_error/num_present/Select:mean_squared_error/num_present/broadcast_weights/ones_like*
T0
�
$mean_squared_error/num_present/ConstConstD^mean_squared_error/assert_broadcastable/static_scalar_check_success*
valueB"       *
dtype0
�
mean_squared_error/num_presentSum0mean_squared_error/num_present/broadcast_weights$mean_squared_error/num_present/Const*

Tidx0*
	keep_dims( *
T0
�
mean_squared_error/Const_1ConstD^mean_squared_error/assert_broadcastable/static_scalar_check_success*
valueB *
dtype0
y
mean_squared_error/Sum_1Summean_squared_error/Summean_squared_error/Const_1*

Tidx0*
	keep_dims( *
T0
�
mean_squared_error/Greater/yConstD^mean_squared_error/assert_broadcastable/static_scalar_check_success*
valueB
 *    *
dtype0
l
mean_squared_error/GreaterGreatermean_squared_error/num_presentmean_squared_error/Greater/y*
T0
�
mean_squared_error/Equal/yConstD^mean_squared_error/assert_broadcastable/static_scalar_check_success*
dtype0*
valueB
 *    
f
mean_squared_error/EqualEqualmean_squared_error/num_presentmean_squared_error/Equal/y*
T0
�
"mean_squared_error/ones_like/ShapeConstD^mean_squared_error/assert_broadcastable/static_scalar_check_success*
dtype0*
valueB 
�
"mean_squared_error/ones_like/ConstConstD^mean_squared_error/assert_broadcastable/static_scalar_check_success*
valueB
 *  �?*
dtype0
�
mean_squared_error/ones_likeFill"mean_squared_error/ones_like/Shape"mean_squared_error/ones_like/Const*
T0*

index_type0
�
mean_squared_error/SelectSelectmean_squared_error/Equalmean_squared_error/ones_likemean_squared_error/num_present*
T0
_
mean_squared_error/divRealDivmean_squared_error/Sum_1mean_squared_error/Select*
T0
�
mean_squared_error/zeros_likeConstD^mean_squared_error/assert_broadcastable/static_scalar_check_success*
valueB
 *    *
dtype0
~
mean_squared_error/valueSelectmean_squared_error/Greatermean_squared_error/divmean_squared_error/zeros_like*
T0
8
gradients/ShapeConst*
valueB *
dtype0
@
gradients/grad_ys_0Const*
valueB
 *  �?*
dtype0
W
gradients/FillFillgradients/Shapegradients/grad_ys_0*
T0*

index_type0
_
2gradients/mean_squared_error/value_grad/zeros_likeConst*
valueB
 *    *
dtype0
�
.gradients/mean_squared_error/value_grad/SelectSelectmean_squared_error/Greatergradients/Fill2gradients/mean_squared_error/value_grad/zeros_like*
T0
�
0gradients/mean_squared_error/value_grad/Select_1Selectmean_squared_error/Greater2gradients/mean_squared_error/value_grad/zeros_likegradients/Fill*
T0
�
8gradients/mean_squared_error/value_grad/tuple/group_depsNoOp/^gradients/mean_squared_error/value_grad/Select1^gradients/mean_squared_error/value_grad/Select_1
�
@gradients/mean_squared_error/value_grad/tuple/control_dependencyIdentity.gradients/mean_squared_error/value_grad/Select9^gradients/mean_squared_error/value_grad/tuple/group_deps*
T0*A
_class7
53loc:@gradients/mean_squared_error/value_grad/Select
�
Bgradients/mean_squared_error/value_grad/tuple/control_dependency_1Identity0gradients/mean_squared_error/value_grad/Select_19^gradients/mean_squared_error/value_grad/tuple/group_deps*
T0*C
_class9
75loc:@gradients/mean_squared_error/value_grad/Select_1
T
+gradients/mean_squared_error/div_grad/ShapeConst*
valueB *
dtype0
V
-gradients/mean_squared_error/div_grad/Shape_1Const*
dtype0*
valueB 
�
;gradients/mean_squared_error/div_grad/BroadcastGradientArgsBroadcastGradientArgs+gradients/mean_squared_error/div_grad/Shape-gradients/mean_squared_error/div_grad/Shape_1*
T0
�
-gradients/mean_squared_error/div_grad/RealDivRealDiv@gradients/mean_squared_error/value_grad/tuple/control_dependencymean_squared_error/Select*
T0
�
)gradients/mean_squared_error/div_grad/SumSum-gradients/mean_squared_error/div_grad/RealDiv;gradients/mean_squared_error/div_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
-gradients/mean_squared_error/div_grad/ReshapeReshape)gradients/mean_squared_error/div_grad/Sum+gradients/mean_squared_error/div_grad/Shape*
T0*
Tshape0
S
)gradients/mean_squared_error/div_grad/NegNegmean_squared_error/Sum_1*
T0
�
/gradients/mean_squared_error/div_grad/RealDiv_1RealDiv)gradients/mean_squared_error/div_grad/Negmean_squared_error/Select*
T0
�
/gradients/mean_squared_error/div_grad/RealDiv_2RealDiv/gradients/mean_squared_error/div_grad/RealDiv_1mean_squared_error/Select*
T0
�
)gradients/mean_squared_error/div_grad/mulMul@gradients/mean_squared_error/value_grad/tuple/control_dependency/gradients/mean_squared_error/div_grad/RealDiv_2*
T0
�
+gradients/mean_squared_error/div_grad/Sum_1Sum)gradients/mean_squared_error/div_grad/mul=gradients/mean_squared_error/div_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
/gradients/mean_squared_error/div_grad/Reshape_1Reshape+gradients/mean_squared_error/div_grad/Sum_1-gradients/mean_squared_error/div_grad/Shape_1*
T0*
Tshape0
�
6gradients/mean_squared_error/div_grad/tuple/group_depsNoOp.^gradients/mean_squared_error/div_grad/Reshape0^gradients/mean_squared_error/div_grad/Reshape_1
�
>gradients/mean_squared_error/div_grad/tuple/control_dependencyIdentity-gradients/mean_squared_error/div_grad/Reshape7^gradients/mean_squared_error/div_grad/tuple/group_deps*
T0*@
_class6
42loc:@gradients/mean_squared_error/div_grad/Reshape
�
@gradients/mean_squared_error/div_grad/tuple/control_dependency_1Identity/gradients/mean_squared_error/div_grad/Reshape_17^gradients/mean_squared_error/div_grad/tuple/group_deps*
T0*B
_class8
64loc:@gradients/mean_squared_error/div_grad/Reshape_1
^
5gradients/mean_squared_error/Sum_1_grad/Reshape/shapeConst*
valueB *
dtype0
�
/gradients/mean_squared_error/Sum_1_grad/ReshapeReshape>gradients/mean_squared_error/div_grad/tuple/control_dependency5gradients/mean_squared_error/Sum_1_grad/Reshape/shape*
T0*
Tshape0
V
-gradients/mean_squared_error/Sum_1_grad/ConstConst*
valueB *
dtype0
�
,gradients/mean_squared_error/Sum_1_grad/TileTile/gradients/mean_squared_error/Sum_1_grad/Reshape-gradients/mean_squared_error/Sum_1_grad/Const*
T0*

Tmultiples0
h
3gradients/mean_squared_error/Sum_grad/Reshape/shapeConst*
valueB"      *
dtype0
�
-gradients/mean_squared_error/Sum_grad/ReshapeReshape,gradients/mean_squared_error/Sum_1_grad/Tile3gradients/mean_squared_error/Sum_grad/Reshape/shape*
T0*
Tshape0
e
+gradients/mean_squared_error/Sum_grad/ShapeShapemean_squared_error/Mul*
out_type0*
T0
�
*gradients/mean_squared_error/Sum_grad/TileTile-gradients/mean_squared_error/Sum_grad/Reshape+gradients/mean_squared_error/Sum_grad/Shape*

Tmultiples0*
T0
s
+gradients/mean_squared_error/Mul_grad/ShapeShape$mean_squared_error/SquaredDifference*
T0*
out_type0
V
-gradients/mean_squared_error/Mul_grad/Shape_1Const*
valueB *
dtype0
�
;gradients/mean_squared_error/Mul_grad/BroadcastGradientArgsBroadcastGradientArgs+gradients/mean_squared_error/Mul_grad/Shape-gradients/mean_squared_error/Mul_grad/Shape_1*
T0
�
)gradients/mean_squared_error/Mul_grad/MulMul*gradients/mean_squared_error/Sum_grad/Tilemean_squared_error/ToFloat/x*
T0
�
)gradients/mean_squared_error/Mul_grad/SumSum)gradients/mean_squared_error/Mul_grad/Mul;gradients/mean_squared_error/Mul_grad/BroadcastGradientArgs*

Tidx0*
	keep_dims( *
T0
�
-gradients/mean_squared_error/Mul_grad/ReshapeReshape)gradients/mean_squared_error/Mul_grad/Sum+gradients/mean_squared_error/Mul_grad/Shape*
T0*
Tshape0
�
+gradients/mean_squared_error/Mul_grad/Mul_1Mul$mean_squared_error/SquaredDifference*gradients/mean_squared_error/Sum_grad/Tile*
T0
�
+gradients/mean_squared_error/Mul_grad/Sum_1Sum+gradients/mean_squared_error/Mul_grad/Mul_1=gradients/mean_squared_error/Mul_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
/gradients/mean_squared_error/Mul_grad/Reshape_1Reshape+gradients/mean_squared_error/Mul_grad/Sum_1-gradients/mean_squared_error/Mul_grad/Shape_1*
Tshape0*
T0
�
6gradients/mean_squared_error/Mul_grad/tuple/group_depsNoOp.^gradients/mean_squared_error/Mul_grad/Reshape0^gradients/mean_squared_error/Mul_grad/Reshape_1
�
>gradients/mean_squared_error/Mul_grad/tuple/control_dependencyIdentity-gradients/mean_squared_error/Mul_grad/Reshape7^gradients/mean_squared_error/Mul_grad/tuple/group_deps*@
_class6
42loc:@gradients/mean_squared_error/Mul_grad/Reshape*
T0
�
@gradients/mean_squared_error/Mul_grad/tuple/control_dependency_1Identity/gradients/mean_squared_error/Mul_grad/Reshape_17^gradients/mean_squared_error/Mul_grad/tuple/group_deps*
T0*B
_class8
64loc:@gradients/mean_squared_error/Mul_grad/Reshape_1
k
9gradients/mean_squared_error/SquaredDifference_grad/ShapeShapeoutput/BiasAdd*
T0*
out_type0
e
;gradients/mean_squared_error/SquaredDifference_grad/Shape_1Shapetarget*
T0*
out_type0
�
Igradients/mean_squared_error/SquaredDifference_grad/BroadcastGradientArgsBroadcastGradientArgs9gradients/mean_squared_error/SquaredDifference_grad/Shape;gradients/mean_squared_error/SquaredDifference_grad/Shape_1*
T0
�
:gradients/mean_squared_error/SquaredDifference_grad/scalarConst?^gradients/mean_squared_error/Mul_grad/tuple/control_dependency*
valueB
 *   @*
dtype0
�
7gradients/mean_squared_error/SquaredDifference_grad/mulMul:gradients/mean_squared_error/SquaredDifference_grad/scalar>gradients/mean_squared_error/Mul_grad/tuple/control_dependency*
T0
�
7gradients/mean_squared_error/SquaredDifference_grad/subSuboutput/BiasAddtarget?^gradients/mean_squared_error/Mul_grad/tuple/control_dependency*
T0
�
9gradients/mean_squared_error/SquaredDifference_grad/mul_1Mul7gradients/mean_squared_error/SquaredDifference_grad/mul7gradients/mean_squared_error/SquaredDifference_grad/sub*
T0
�
7gradients/mean_squared_error/SquaredDifference_grad/SumSum9gradients/mean_squared_error/SquaredDifference_grad/mul_1Igradients/mean_squared_error/SquaredDifference_grad/BroadcastGradientArgs*

Tidx0*
	keep_dims( *
T0
�
;gradients/mean_squared_error/SquaredDifference_grad/ReshapeReshape7gradients/mean_squared_error/SquaredDifference_grad/Sum9gradients/mean_squared_error/SquaredDifference_grad/Shape*
Tshape0*
T0
�
9gradients/mean_squared_error/SquaredDifference_grad/Sum_1Sum9gradients/mean_squared_error/SquaredDifference_grad/mul_1Kgradients/mean_squared_error/SquaredDifference_grad/BroadcastGradientArgs:1*

Tidx0*
	keep_dims( *
T0
�
=gradients/mean_squared_error/SquaredDifference_grad/Reshape_1Reshape9gradients/mean_squared_error/SquaredDifference_grad/Sum_1;gradients/mean_squared_error/SquaredDifference_grad/Shape_1*
T0*
Tshape0
�
7gradients/mean_squared_error/SquaredDifference_grad/NegNeg=gradients/mean_squared_error/SquaredDifference_grad/Reshape_1*
T0
�
Dgradients/mean_squared_error/SquaredDifference_grad/tuple/group_depsNoOp8^gradients/mean_squared_error/SquaredDifference_grad/Neg<^gradients/mean_squared_error/SquaredDifference_grad/Reshape
�
Lgradients/mean_squared_error/SquaredDifference_grad/tuple/control_dependencyIdentity;gradients/mean_squared_error/SquaredDifference_grad/ReshapeE^gradients/mean_squared_error/SquaredDifference_grad/tuple/group_deps*
T0*N
_classD
B@loc:@gradients/mean_squared_error/SquaredDifference_grad/Reshape
�
Ngradients/mean_squared_error/SquaredDifference_grad/tuple/control_dependency_1Identity7gradients/mean_squared_error/SquaredDifference_grad/NegE^gradients/mean_squared_error/SquaredDifference_grad/tuple/group_deps*
T0*J
_class@
><loc:@gradients/mean_squared_error/SquaredDifference_grad/Neg
�
)gradients/output/BiasAdd_grad/BiasAddGradBiasAddGradLgradients/mean_squared_error/SquaredDifference_grad/tuple/control_dependency*
data_formatNHWC*
T0
�
.gradients/output/BiasAdd_grad/tuple/group_depsNoOpM^gradients/mean_squared_error/SquaredDifference_grad/tuple/control_dependency*^gradients/output/BiasAdd_grad/BiasAddGrad
�
6gradients/output/BiasAdd_grad/tuple/control_dependencyIdentityLgradients/mean_squared_error/SquaredDifference_grad/tuple/control_dependency/^gradients/output/BiasAdd_grad/tuple/group_deps*
T0*N
_classD
B@loc:@gradients/mean_squared_error/SquaredDifference_grad/Reshape
�
8gradients/output/BiasAdd_grad/tuple/control_dependency_1Identity)gradients/output/BiasAdd_grad/BiasAddGrad/^gradients/output/BiasAdd_grad/tuple/group_deps*
T0*<
_class2
0.loc:@gradients/output/BiasAdd_grad/BiasAddGrad
�
#gradients/output/MatMul_grad/MatMulMatMul6gradients/output/BiasAdd_grad/tuple/control_dependencyoutput/kernel/read*
T0*
transpose_a( *
transpose_b(
�
%gradients/output/MatMul_grad/MatMul_1MatMulh2/Relu6gradients/output/BiasAdd_grad/tuple/control_dependency*
transpose_a(*
transpose_b( *
T0
�
-gradients/output/MatMul_grad/tuple/group_depsNoOp$^gradients/output/MatMul_grad/MatMul&^gradients/output/MatMul_grad/MatMul_1
�
5gradients/output/MatMul_grad/tuple/control_dependencyIdentity#gradients/output/MatMul_grad/MatMul.^gradients/output/MatMul_grad/tuple/group_deps*
T0*6
_class,
*(loc:@gradients/output/MatMul_grad/MatMul
�
7gradients/output/MatMul_grad/tuple/control_dependency_1Identity%gradients/output/MatMul_grad/MatMul_1.^gradients/output/MatMul_grad/tuple/group_deps*8
_class.
,*loc:@gradients/output/MatMul_grad/MatMul_1*
T0
t
gradients/h2/Relu_grad/ReluGradReluGrad5gradients/output/MatMul_grad/tuple/control_dependencyh2/Relu*
T0
u
%gradients/h2/BiasAdd_grad/BiasAddGradBiasAddGradgradients/h2/Relu_grad/ReluGrad*
T0*
data_formatNHWC
|
*gradients/h2/BiasAdd_grad/tuple/group_depsNoOp&^gradients/h2/BiasAdd_grad/BiasAddGrad ^gradients/h2/Relu_grad/ReluGrad
�
2gradients/h2/BiasAdd_grad/tuple/control_dependencyIdentitygradients/h2/Relu_grad/ReluGrad+^gradients/h2/BiasAdd_grad/tuple/group_deps*
T0*2
_class(
&$loc:@gradients/h2/Relu_grad/ReluGrad
�
4gradients/h2/BiasAdd_grad/tuple/control_dependency_1Identity%gradients/h2/BiasAdd_grad/BiasAddGrad+^gradients/h2/BiasAdd_grad/tuple/group_deps*
T0*8
_class.
,*loc:@gradients/h2/BiasAdd_grad/BiasAddGrad
�
gradients/h2/MatMul_grad/MatMulMatMul2gradients/h2/BiasAdd_grad/tuple/control_dependencyh2/kernel/read*
T0*
transpose_a( *
transpose_b(
�
!gradients/h2/MatMul_grad/MatMul_1MatMulh1/Relu2gradients/h2/BiasAdd_grad/tuple/control_dependency*
transpose_b( *
T0*
transpose_a(
w
)gradients/h2/MatMul_grad/tuple/group_depsNoOp ^gradients/h2/MatMul_grad/MatMul"^gradients/h2/MatMul_grad/MatMul_1
�
1gradients/h2/MatMul_grad/tuple/control_dependencyIdentitygradients/h2/MatMul_grad/MatMul*^gradients/h2/MatMul_grad/tuple/group_deps*
T0*2
_class(
&$loc:@gradients/h2/MatMul_grad/MatMul
�
3gradients/h2/MatMul_grad/tuple/control_dependency_1Identity!gradients/h2/MatMul_grad/MatMul_1*^gradients/h2/MatMul_grad/tuple/group_deps*
T0*4
_class*
(&loc:@gradients/h2/MatMul_grad/MatMul_1
p
gradients/h1/Relu_grad/ReluGradReluGrad1gradients/h2/MatMul_grad/tuple/control_dependencyh1/Relu*
T0
u
%gradients/h1/BiasAdd_grad/BiasAddGradBiasAddGradgradients/h1/Relu_grad/ReluGrad*
data_formatNHWC*
T0
|
*gradients/h1/BiasAdd_grad/tuple/group_depsNoOp&^gradients/h1/BiasAdd_grad/BiasAddGrad ^gradients/h1/Relu_grad/ReluGrad
�
2gradients/h1/BiasAdd_grad/tuple/control_dependencyIdentitygradients/h1/Relu_grad/ReluGrad+^gradients/h1/BiasAdd_grad/tuple/group_deps*
T0*2
_class(
&$loc:@gradients/h1/Relu_grad/ReluGrad
�
4gradients/h1/BiasAdd_grad/tuple/control_dependency_1Identity%gradients/h1/BiasAdd_grad/BiasAddGrad+^gradients/h1/BiasAdd_grad/tuple/group_deps*
T0*8
_class.
,*loc:@gradients/h1/BiasAdd_grad/BiasAddGrad
�
gradients/h1/MatMul_grad/MatMulMatMul2gradients/h1/BiasAdd_grad/tuple/control_dependencyh1/kernel/read*
T0*
transpose_a( *
transpose_b(
�
!gradients/h1/MatMul_grad/MatMul_1MatMulinput2gradients/h1/BiasAdd_grad/tuple/control_dependency*
T0*
transpose_a(*
transpose_b( 
w
)gradients/h1/MatMul_grad/tuple/group_depsNoOp ^gradients/h1/MatMul_grad/MatMul"^gradients/h1/MatMul_grad/MatMul_1
�
1gradients/h1/MatMul_grad/tuple/control_dependencyIdentitygradients/h1/MatMul_grad/MatMul*^gradients/h1/MatMul_grad/tuple/group_deps*
T0*2
_class(
&$loc:@gradients/h1/MatMul_grad/MatMul
�
3gradients/h1/MatMul_grad/tuple/control_dependency_1Identity!gradients/h1/MatMul_grad/MatMul_1*^gradients/h1/MatMul_grad/tuple/group_deps*
T0*4
_class*
(&loc:@gradients/h1/MatMul_grad/MatMul_1
b
beta1_power/initial_valueConst*
dtype0*
valueB
 *fff?*
_class
loc:@h1/bias
s
beta1_power
VariableV2*
shape: *
shared_name *
_class
loc:@h1/bias*
dtype0*
	container 
�
beta1_power/AssignAssignbeta1_powerbeta1_power/initial_value*
use_locking(*
T0*
_class
loc:@h1/bias*
validate_shape(
N
beta1_power/readIdentitybeta1_power*
_class
loc:@h1/bias*
T0
b
beta2_power/initial_valueConst*
valueB
 *w�?*
_class
loc:@h1/bias*
dtype0
s
beta2_power
VariableV2*
shape: *
shared_name *
_class
loc:@h1/bias*
dtype0*
	container 
�
beta2_power/AssignAssignbeta2_powerbeta2_power/initial_value*
use_locking(*
T0*
_class
loc:@h1/bias*
validate_shape(
N
beta2_power/readIdentitybeta2_power*
T0*
_class
loc:@h1/bias
t
 h1/kernel/Adam/Initializer/zerosConst*
dtype0*
valueB	�*    *
_class
loc:@h1/kernel
�
h1/kernel/Adam
VariableV2*
shape:	�*
shared_name *
_class
loc:@h1/kernel*
dtype0*
	container 
�
h1/kernel/Adam/AssignAssignh1/kernel/Adam h1/kernel/Adam/Initializer/zeros*
_class
loc:@h1/kernel*
validate_shape(*
use_locking(*
T0
V
h1/kernel/Adam/readIdentityh1/kernel/Adam*
_class
loc:@h1/kernel*
T0
v
"h1/kernel/Adam_1/Initializer/zerosConst*
valueB	�*    *
_class
loc:@h1/kernel*
dtype0
�
h1/kernel/Adam_1
VariableV2*
shared_name *
_class
loc:@h1/kernel*
dtype0*
	container *
shape:	�
�
h1/kernel/Adam_1/AssignAssignh1/kernel/Adam_1"h1/kernel/Adam_1/Initializer/zeros*
_class
loc:@h1/kernel*
validate_shape(*
use_locking(*
T0
Z
h1/kernel/Adam_1/readIdentityh1/kernel/Adam_1*
T0*
_class
loc:@h1/kernel
l
h1/bias/Adam/Initializer/zerosConst*
valueB�*    *
_class
loc:@h1/bias*
dtype0
y
h1/bias/Adam
VariableV2*
shared_name *
_class
loc:@h1/bias*
dtype0*
	container *
shape:�
�
h1/bias/Adam/AssignAssignh1/bias/Adamh1/bias/Adam/Initializer/zeros*
use_locking(*
T0*
_class
loc:@h1/bias*
validate_shape(
P
h1/bias/Adam/readIdentityh1/bias/Adam*
T0*
_class
loc:@h1/bias
n
 h1/bias/Adam_1/Initializer/zerosConst*
dtype0*
valueB�*    *
_class
loc:@h1/bias
{
h1/bias/Adam_1
VariableV2*
	container *
shape:�*
shared_name *
_class
loc:@h1/bias*
dtype0
�
h1/bias/Adam_1/AssignAssignh1/bias/Adam_1 h1/bias/Adam_1/Initializer/zeros*
_class
loc:@h1/bias*
validate_shape(*
use_locking(*
T0
T
h1/bias/Adam_1/readIdentityh1/bias/Adam_1*
T0*
_class
loc:@h1/bias
�
0h2/kernel/Adam/Initializer/zeros/shape_as_tensorConst*
valueB"�  ,  *
_class
loc:@h2/kernel*
dtype0
q
&h2/kernel/Adam/Initializer/zeros/ConstConst*
valueB
 *    *
_class
loc:@h2/kernel*
dtype0
�
 h2/kernel/Adam/Initializer/zerosFill0h2/kernel/Adam/Initializer/zeros/shape_as_tensor&h2/kernel/Adam/Initializer/zeros/Const*

index_type0*
_class
loc:@h2/kernel*
T0
�
h2/kernel/Adam
VariableV2*
_class
loc:@h2/kernel*
dtype0*
	container *
shape:
��*
shared_name 
�
h2/kernel/Adam/AssignAssignh2/kernel/Adam h2/kernel/Adam/Initializer/zeros*
use_locking(*
T0*
_class
loc:@h2/kernel*
validate_shape(
V
h2/kernel/Adam/readIdentityh2/kernel/Adam*
_class
loc:@h2/kernel*
T0
�
2h2/kernel/Adam_1/Initializer/zeros/shape_as_tensorConst*
valueB"�  ,  *
_class
loc:@h2/kernel*
dtype0
s
(h2/kernel/Adam_1/Initializer/zeros/ConstConst*
valueB
 *    *
_class
loc:@h2/kernel*
dtype0
�
"h2/kernel/Adam_1/Initializer/zerosFill2h2/kernel/Adam_1/Initializer/zeros/shape_as_tensor(h2/kernel/Adam_1/Initializer/zeros/Const*

index_type0*
_class
loc:@h2/kernel*
T0
�
h2/kernel/Adam_1
VariableV2*
shape:
��*
shared_name *
_class
loc:@h2/kernel*
dtype0*
	container 
�
h2/kernel/Adam_1/AssignAssignh2/kernel/Adam_1"h2/kernel/Adam_1/Initializer/zeros*
use_locking(*
T0*
_class
loc:@h2/kernel*
validate_shape(
Z
h2/kernel/Adam_1/readIdentityh2/kernel/Adam_1*
_class
loc:@h2/kernel*
T0
l
h2/bias/Adam/Initializer/zerosConst*
valueB�*    *
_class
loc:@h2/bias*
dtype0
y
h2/bias/Adam
VariableV2*
dtype0*
	container *
shape:�*
shared_name *
_class
loc:@h2/bias
�
h2/bias/Adam/AssignAssignh2/bias/Adamh2/bias/Adam/Initializer/zeros*
use_locking(*
T0*
_class
loc:@h2/bias*
validate_shape(
P
h2/bias/Adam/readIdentityh2/bias/Adam*
T0*
_class
loc:@h2/bias
n
 h2/bias/Adam_1/Initializer/zerosConst*
valueB�*    *
_class
loc:@h2/bias*
dtype0
{
h2/bias/Adam_1
VariableV2*
shared_name *
_class
loc:@h2/bias*
dtype0*
	container *
shape:�
�
h2/bias/Adam_1/AssignAssignh2/bias/Adam_1 h2/bias/Adam_1/Initializer/zeros*
_class
loc:@h2/bias*
validate_shape(*
use_locking(*
T0
T
h2/bias/Adam_1/readIdentityh2/bias/Adam_1*
T0*
_class
loc:@h2/bias
|
$output/kernel/Adam/Initializer/zerosConst*
valueB	�*    * 
_class
loc:@output/kernel*
dtype0
�
output/kernel/Adam
VariableV2*
dtype0*
	container *
shape:	�*
shared_name * 
_class
loc:@output/kernel
�
output/kernel/Adam/AssignAssignoutput/kernel/Adam$output/kernel/Adam/Initializer/zeros* 
_class
loc:@output/kernel*
validate_shape(*
use_locking(*
T0
b
output/kernel/Adam/readIdentityoutput/kernel/Adam*
T0* 
_class
loc:@output/kernel
~
&output/kernel/Adam_1/Initializer/zerosConst*
valueB	�*    * 
_class
loc:@output/kernel*
dtype0
�
output/kernel/Adam_1
VariableV2* 
_class
loc:@output/kernel*
dtype0*
	container *
shape:	�*
shared_name 
�
output/kernel/Adam_1/AssignAssignoutput/kernel/Adam_1&output/kernel/Adam_1/Initializer/zeros*
use_locking(*
T0* 
_class
loc:@output/kernel*
validate_shape(
f
output/kernel/Adam_1/readIdentityoutput/kernel/Adam_1*
T0* 
_class
loc:@output/kernel
s
"output/bias/Adam/Initializer/zerosConst*
dtype0*
valueB*    *
_class
loc:@output/bias
�
output/bias/Adam
VariableV2*
_class
loc:@output/bias*
dtype0*
	container *
shape:*
shared_name 
�
output/bias/Adam/AssignAssignoutput/bias/Adam"output/bias/Adam/Initializer/zeros*
use_locking(*
T0*
_class
loc:@output/bias*
validate_shape(
\
output/bias/Adam/readIdentityoutput/bias/Adam*
T0*
_class
loc:@output/bias
u
$output/bias/Adam_1/Initializer/zerosConst*
valueB*    *
_class
loc:@output/bias*
dtype0
�
output/bias/Adam_1
VariableV2*
shape:*
shared_name *
_class
loc:@output/bias*
dtype0*
	container 
�
output/bias/Adam_1/AssignAssignoutput/bias/Adam_1$output/bias/Adam_1/Initializer/zeros*
T0*
_class
loc:@output/bias*
validate_shape(*
use_locking(
`
output/bias/Adam_1/readIdentityoutput/bias/Adam_1*
T0*
_class
loc:@output/bias
A
update/learning_rateConst*
valueB
 *o�:*
dtype0
9
update/beta1Const*
valueB
 *fff?*
dtype0
9
update/beta2Const*
valueB
 *w�?*
dtype0
;
update/epsilonConst*
valueB
 *w�+2*
dtype0
�
!update/update_h1/kernel/ApplyAdam	ApplyAdam	h1/kernelh1/kernel/Adamh1/kernel/Adam_1beta1_power/readbeta2_power/readupdate/learning_rateupdate/beta1update/beta2update/epsilon3gradients/h1/MatMul_grad/tuple/control_dependency_1*
use_locking( *
T0*
_class
loc:@h1/kernel*
use_nesterov( 
�
update/update_h1/bias/ApplyAdam	ApplyAdamh1/biash1/bias/Adamh1/bias/Adam_1beta1_power/readbeta2_power/readupdate/learning_rateupdate/beta1update/beta2update/epsilon4gradients/h1/BiasAdd_grad/tuple/control_dependency_1*
use_locking( *
T0*
_class
loc:@h1/bias*
use_nesterov( 
�
!update/update_h2/kernel/ApplyAdam	ApplyAdam	h2/kernelh2/kernel/Adamh2/kernel/Adam_1beta1_power/readbeta2_power/readupdate/learning_rateupdate/beta1update/beta2update/epsilon3gradients/h2/MatMul_grad/tuple/control_dependency_1*
T0*
_class
loc:@h2/kernel*
use_nesterov( *
use_locking( 
�
update/update_h2/bias/ApplyAdam	ApplyAdamh2/biash2/bias/Adamh2/bias/Adam_1beta1_power/readbeta2_power/readupdate/learning_rateupdate/beta1update/beta2update/epsilon4gradients/h2/BiasAdd_grad/tuple/control_dependency_1*
T0*
_class
loc:@h2/bias*
use_nesterov( *
use_locking( 
�
%update/update_output/kernel/ApplyAdam	ApplyAdamoutput/kerneloutput/kernel/Adamoutput/kernel/Adam_1beta1_power/readbeta2_power/readupdate/learning_rateupdate/beta1update/beta2update/epsilon7gradients/output/MatMul_grad/tuple/control_dependency_1*
use_locking( *
T0* 
_class
loc:@output/kernel*
use_nesterov( 
�
#update/update_output/bias/ApplyAdam	ApplyAdamoutput/biasoutput/bias/Adamoutput/bias/Adam_1beta1_power/readbeta2_power/readupdate/learning_rateupdate/beta1update/beta2update/epsilon8gradients/output/BiasAdd_grad/tuple/control_dependency_1*
use_locking( *
T0*
_class
loc:@output/bias*
use_nesterov( 
�

update/mulMulbeta1_power/readupdate/beta1 ^update/update_h1/bias/ApplyAdam"^update/update_h1/kernel/ApplyAdam ^update/update_h2/bias/ApplyAdam"^update/update_h2/kernel/ApplyAdam$^update/update_output/bias/ApplyAdam&^update/update_output/kernel/ApplyAdam*
T0*
_class
loc:@h1/bias
~
update/AssignAssignbeta1_power
update/mul*
use_locking( *
T0*
_class
loc:@h1/bias*
validate_shape(
�
update/mul_1Mulbeta2_power/readupdate/beta2 ^update/update_h1/bias/ApplyAdam"^update/update_h1/kernel/ApplyAdam ^update/update_h2/bias/ApplyAdam"^update/update_h2/kernel/ApplyAdam$^update/update_output/bias/ApplyAdam&^update/update_output/kernel/ApplyAdam*
T0*
_class
loc:@h1/bias
�
update/Assign_1Assignbeta2_powerupdate/mul_1*
use_locking( *
T0*
_class
loc:@h1/bias*
validate_shape(
�
updateNoOp^update/Assign^update/Assign_1 ^update/update_h1/bias/ApplyAdam"^update/update_h1/kernel/ApplyAdam ^update/update_h2/bias/ApplyAdam"^update/update_h2/kernel/ApplyAdam$^update/update_output/bias/ApplyAdam&^update/update_output/kernel/ApplyAdam
=
PlaceholderPlaceholder*
shape:	�*
dtype0
x
AssignAssign	h1/kernelPlaceholder*
use_locking(*
T0*
_class
loc:@h1/kernel*
validate_shape(
;
Placeholder_1Placeholder*
shape:�*
dtype0
x
Assign_1Assignh1/biasPlaceholder_1*
use_locking(*
T0*
_class
loc:@h1/bias*
validate_shape(
@
Placeholder_2Placeholder*
dtype0*
shape:
��
|
Assign_2Assign	h2/kernelPlaceholder_2*
use_locking(*
T0*
_class
loc:@h2/kernel*
validate_shape(
;
Placeholder_3Placeholder*
shape:�*
dtype0
x
Assign_3Assignh2/biasPlaceholder_3*
use_locking(*
T0*
_class
loc:@h2/bias*
validate_shape(
?
Placeholder_4Placeholder*
shape:	�*
dtype0
�
Assign_4Assignoutput/kernelPlaceholder_4* 
_class
loc:@output/kernel*
validate_shape(*
use_locking(*
T0
:
Placeholder_5Placeholder*
dtype0*
shape:
�
Assign_5Assignoutput/biasPlaceholder_5*
_class
loc:@output/bias*
validate_shape(*
use_locking(*
T0
d
IsVariableInitializedIsVariableInitialized	h1/kernel*
dtype0*
_class
loc:@h1/kernel
b
IsVariableInitialized_1IsVariableInitializedh1/bias*
_class
loc:@h1/bias*
dtype0
f
IsVariableInitialized_2IsVariableInitialized	h2/kernel*
_class
loc:@h2/kernel*
dtype0
b
IsVariableInitialized_3IsVariableInitializedh2/bias*
_class
loc:@h2/bias*
dtype0
n
IsVariableInitialized_4IsVariableInitializedoutput/kernel* 
_class
loc:@output/kernel*
dtype0
j
IsVariableInitialized_5IsVariableInitializedoutput/bias*
_class
loc:@output/bias*
dtype0
f
IsVariableInitialized_6IsVariableInitializedbeta1_power*
_class
loc:@h1/bias*
dtype0
f
IsVariableInitialized_7IsVariableInitializedbeta2_power*
_class
loc:@h1/bias*
dtype0
k
IsVariableInitialized_8IsVariableInitializedh1/kernel/Adam*
_class
loc:@h1/kernel*
dtype0
m
IsVariableInitialized_9IsVariableInitializedh1/kernel/Adam_1*
_class
loc:@h1/kernel*
dtype0
h
IsVariableInitialized_10IsVariableInitializedh1/bias/Adam*
_class
loc:@h1/bias*
dtype0
j
IsVariableInitialized_11IsVariableInitializedh1/bias/Adam_1*
_class
loc:@h1/bias*
dtype0
l
IsVariableInitialized_12IsVariableInitializedh2/kernel/Adam*
_class
loc:@h2/kernel*
dtype0
n
IsVariableInitialized_13IsVariableInitializedh2/kernel/Adam_1*
_class
loc:@h2/kernel*
dtype0
h
IsVariableInitialized_14IsVariableInitializedh2/bias/Adam*
_class
loc:@h2/bias*
dtype0
j
IsVariableInitialized_15IsVariableInitializedh2/bias/Adam_1*
_class
loc:@h2/bias*
dtype0
t
IsVariableInitialized_16IsVariableInitializedoutput/kernel/Adam*
dtype0* 
_class
loc:@output/kernel
v
IsVariableInitialized_17IsVariableInitializedoutput/kernel/Adam_1* 
_class
loc:@output/kernel*
dtype0
p
IsVariableInitialized_18IsVariableInitializedoutput/bias/Adam*
_class
loc:@output/bias*
dtype0
r
IsVariableInitialized_19IsVariableInitializedoutput/bias/Adam_1*
_class
loc:@output/bias*
dtype0
�
initNoOp^beta1_power/Assign^beta2_power/Assign^h1/bias/Adam/Assign^h1/bias/Adam_1/Assign^h1/bias/Assign^h1/kernel/Adam/Assign^h1/kernel/Adam_1/Assign^h1/kernel/Assign^h2/bias/Adam/Assign^h2/bias/Adam_1/Assign^h2/bias/Assign^h2/kernel/Adam/Assign^h2/kernel/Adam_1/Assign^h2/kernel/Assign^output/bias/Adam/Assign^output/bias/Adam_1/Assign^output/bias/Assign^output/kernel/Adam/Assign^output/kernel/Adam_1/Assign^output/kernel/Assign"