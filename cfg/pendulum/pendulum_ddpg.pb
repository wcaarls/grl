
>
s_inPlaceholder*
dtype0*
shape:ÿÿÿÿÿÿÿÿÿ
R
h_common/random_uniform/shapeConst*
valueB"     *
dtype0
H
h_common/random_uniform/minConst*
valueB
 *ù3ú½*
dtype0
H
h_common/random_uniform/maxConst*
valueB
 *ù3ú=*
dtype0

%h_common/random_uniform/RandomUniformRandomUniformh_common/random_uniform/shape*
seed2Ò²@*
seed±ÿå)*
T0*
dtype0
e
h_common/random_uniform/subSubh_common/random_uniform/maxh_common/random_uniform/min*
T0
o
h_common/random_uniform/mulMul%h_common/random_uniform/RandomUniformh_common/random_uniform/sub*
T0
a
h_common/random_uniformAddh_common/random_uniform/mulh_common/random_uniform/min*
T0
d
h_common/kernel
VariableV2*
dtype0*
	container *
shape:	*
shared_name 
 
h_common/kernel/AssignAssignh_common/kernelh_common/random_uniform*
use_locking(*
T0*"
_class
loc:@h_common/kernel*
validate_shape(
^
h_common/kernel/readIdentityh_common/kernel*
T0*"
_class
loc:@h_common/kernel
@
h_common/ConstConst*
valueB*    *
dtype0
^
h_common/bias
VariableV2*
shape:*
shared_name *
dtype0*
	container 

h_common/bias/AssignAssignh_common/biash_common/Const*
use_locking(*
T0* 
_class
loc:@h_common/bias*
validate_shape(
X
h_common/bias/readIdentityh_common/bias*
T0* 
_class
loc:@h_common/bias
d
h_common/MatMulMatMuls_inh_common/kernel/read*
T0*
transpose_a( *
transpose_b( 
`
h_common/BiasAddBiasAddh_common/MatMulh_common/bias/read*
T0*
data_formatNHWC
0
h_common/ReluReluh_common/BiasAdd*
T0
Q
h_actor/random_uniform/shapeConst*
valueB"  ,  *
dtype0
G
h_actor/random_uniform/minConst*
valueB
 *£½½*
dtype0
G
h_actor/random_uniform/maxConst*
valueB
 *£½=*
dtype0

$h_actor/random_uniform/RandomUniformRandomUniformh_actor/random_uniform/shape*
T0*
dtype0*
seed2¦Ï*
seed±ÿå)
b
h_actor/random_uniform/subSubh_actor/random_uniform/maxh_actor/random_uniform/min*
T0
l
h_actor/random_uniform/mulMul$h_actor/random_uniform/RandomUniformh_actor/random_uniform/sub*
T0
^
h_actor/random_uniformAddh_actor/random_uniform/mulh_actor/random_uniform/min*
T0
d
h_actor/kernel
VariableV2*
	container *
shape:
¬*
shared_name *
dtype0

h_actor/kernel/AssignAssignh_actor/kernelh_actor/random_uniform*
use_locking(*
T0*!
_class
loc:@h_actor/kernel*
validate_shape(
[
h_actor/kernel/readIdentityh_actor/kernel*
T0*!
_class
loc:@h_actor/kernel
?
h_actor/ConstConst*
valueB¬*    *
dtype0
]
h_actor/bias
VariableV2*
shared_name *
dtype0*
	container *
shape:¬

h_actor/bias/AssignAssignh_actor/biash_actor/Const*
T0*
_class
loc:@h_actor/bias*
validate_shape(*
use_locking(
U
h_actor/bias/readIdentityh_actor/bias*
T0*
_class
loc:@h_actor/bias
k
h_actor/MatMulMatMulh_common/Reluh_actor/kernel/read*
T0*
transpose_a( *
transpose_b( 
]
h_actor/BiasAddBiasAddh_actor/MatMulh_actor/bias/read*
T0*
data_formatNHWC
.
h_actor/ReluReluh_actor/BiasAdd*
T0
O
a_raw/random_uniform/shapeConst*
valueB",     *
dtype0
E
a_raw/random_uniform/minConst*
valueB
 * ¾*
dtype0
E
a_raw/random_uniform/maxConst*
dtype0*
valueB
 * >

"a_raw/random_uniform/RandomUniformRandomUniforma_raw/random_uniform/shape*
T0*
dtype0*
seed2åşi*
seed±ÿå)
\
a_raw/random_uniform/subSuba_raw/random_uniform/maxa_raw/random_uniform/min*
T0
f
a_raw/random_uniform/mulMul"a_raw/random_uniform/RandomUniforma_raw/random_uniform/sub*
T0
X
a_raw/random_uniformAdda_raw/random_uniform/mula_raw/random_uniform/min*
T0
a
a_raw/kernel
VariableV2*
	container *
shape:	¬*
shared_name *
dtype0

a_raw/kernel/AssignAssigna_raw/kernela_raw/random_uniform*
use_locking(*
T0*
_class
loc:@a_raw/kernel*
validate_shape(
U
a_raw/kernel/readIdentitya_raw/kernel*
T0*
_class
loc:@a_raw/kernel
<
a_raw/ConstConst*
valueB*    *
dtype0
Z

a_raw/bias
VariableV2*
shared_name *
dtype0*
	container *
shape:

a_raw/bias/AssignAssign
a_raw/biasa_raw/Const*
T0*
_class
loc:@a_raw/bias*
validate_shape(*
use_locking(
O
a_raw/bias/readIdentity
a_raw/bias*
T0*
_class
loc:@a_raw/bias
f
a_raw/MatMulMatMulh_actor/Relua_raw/kernel/read*
transpose_a( *
transpose_b( *
T0
W
a_raw/BiasAddBiasAdda_raw/MatMula_raw/bias/read*
T0*
data_formatNHWC
*

a_raw/TanhTanha_raw/BiasAdd*
T0
8
a_out/mul/xConst*
dtype0*
valueB
 *  @@
2
	a_out/mulMula_out/mul/x
a_raw/Tanh*
T0
K
a_out/PlaceholderPlaceholder*
dtype0*
shape:ÿÿÿÿÿÿÿÿÿ
:
a_out/mul_1/xConst*
valueB
 *  @@*
dtype0
=
a_out/mul_1Mula_out/mul_1/xa_out/Placeholder*
T0
0
StopGradientStopGradient	a_out/mul*
T0
W
a_inPlaceholderWithDefaultStopGradient*
dtype0*
shape:ÿÿÿÿÿÿÿÿÿ
S
h_common2/random_uniform/shapeConst*
valueB"     *
dtype0
I
h_common2/random_uniform/minConst*
valueB
 *ù3ú½*
dtype0
I
h_common2/random_uniform/maxConst*
valueB
 *ù3ú=*
dtype0

&h_common2/random_uniform/RandomUniformRandomUniformh_common2/random_uniform/shape*
dtype0*
seed2ë*
seed±ÿå)*
T0
h
h_common2/random_uniform/subSubh_common2/random_uniform/maxh_common2/random_uniform/min*
T0
r
h_common2/random_uniform/mulMul&h_common2/random_uniform/RandomUniformh_common2/random_uniform/sub*
T0
d
h_common2/random_uniformAddh_common2/random_uniform/mulh_common2/random_uniform/min*
T0
e
h_common2/kernel
VariableV2*
shape:	*
shared_name *
dtype0*
	container 
¤
h_common2/kernel/AssignAssignh_common2/kernelh_common2/random_uniform*#
_class
loc:@h_common2/kernel*
validate_shape(*
use_locking(*
T0
a
h_common2/kernel/readIdentityh_common2/kernel*
T0*#
_class
loc:@h_common2/kernel
A
h_common2/ConstConst*
valueB*    *
dtype0
_
h_common2/bias
VariableV2*
	container *
shape:*
shared_name *
dtype0

h_common2/bias/AssignAssignh_common2/biash_common2/Const*!
_class
loc:@h_common2/bias*
validate_shape(*
use_locking(*
T0
[
h_common2/bias/readIdentityh_common2/bias*
T0*!
_class
loc:@h_common2/bias
f
h_common2/MatMulMatMuls_inh_common2/kernel/read*
T0*
transpose_a( *
transpose_b( 
c
h_common2/BiasAddBiasAddh_common2/MatMulh_common2/bias/read*
T0*
data_formatNHWC
2
h_common2/ReluReluh_common2/BiasAdd*
T0
C
concatenate_1/concat/axisConst*
value	B :*
dtype0
o
concatenate_1/concatConcatV2h_common2/Relua_inconcatenate_1/concat/axis*
T0*
N*

Tidx0
R
h_critic/random_uniform/shapeConst*
valueB"  ,  *
dtype0
H
h_critic/random_uniform/minConst*
valueB
 *y½½*
dtype0
H
h_critic/random_uniform/maxConst*
valueB
 *y½=*
dtype0

%h_critic/random_uniform/RandomUniformRandomUniformh_critic/random_uniform/shape*
T0*
dtype0*
seed2âá+*
seed±ÿå)
e
h_critic/random_uniform/subSubh_critic/random_uniform/maxh_critic/random_uniform/min*
T0
o
h_critic/random_uniform/mulMul%h_critic/random_uniform/RandomUniformh_critic/random_uniform/sub*
T0
a
h_critic/random_uniformAddh_critic/random_uniform/mulh_critic/random_uniform/min*
T0
e
h_critic/kernel
VariableV2*
dtype0*
	container *
shape:
¬*
shared_name 
 
h_critic/kernel/AssignAssignh_critic/kernelh_critic/random_uniform*
use_locking(*
T0*"
_class
loc:@h_critic/kernel*
validate_shape(
^
h_critic/kernel/readIdentityh_critic/kernel*
T0*"
_class
loc:@h_critic/kernel
@
h_critic/ConstConst*
valueB¬*    *
dtype0
^
h_critic/bias
VariableV2*
shared_name *
dtype0*
	container *
shape:¬

h_critic/bias/AssignAssignh_critic/biash_critic/Const*
use_locking(*
T0* 
_class
loc:@h_critic/bias*
validate_shape(
X
h_critic/bias/readIdentityh_critic/bias*
T0* 
_class
loc:@h_critic/bias
t
h_critic/MatMulMatMulconcatenate_1/concath_critic/kernel/read*
transpose_b( *
T0*
transpose_a( 
`
h_critic/BiasAddBiasAddh_critic/MatMulh_critic/bias/read*
data_formatNHWC*
T0
0
h_critic/ReluReluh_critic/BiasAdd*
T0
K
q/random_uniform/shapeConst*
valueB",     *
dtype0
A
q/random_uniform/minConst*
valueB
 * ¾*
dtype0
A
q/random_uniform/maxConst*
valueB
 * >*
dtype0
|
q/random_uniform/RandomUniformRandomUniformq/random_uniform/shape*
dtype0*
seed2¾·*
seed±ÿå)*
T0
P
q/random_uniform/subSubq/random_uniform/maxq/random_uniform/min*
T0
Z
q/random_uniform/mulMulq/random_uniform/RandomUniformq/random_uniform/sub*
T0
L
q/random_uniformAddq/random_uniform/mulq/random_uniform/min*
T0
]
q/kernel
VariableV2*
dtype0*
	container *
shape:	¬*
shared_name 

q/kernel/AssignAssignq/kernelq/random_uniform*
validate_shape(*
use_locking(*
T0*
_class
loc:@q/kernel
I
q/kernel/readIdentityq/kernel*
T0*
_class
loc:@q/kernel
8
q/ConstConst*
valueB*    *
dtype0
V
q/bias
VariableV2*
	container *
shape:*
shared_name *
dtype0
u
q/bias/AssignAssignq/biasq/Const*
use_locking(*
T0*
_class
loc:@q/bias*
validate_shape(
C
q/bias/readIdentityq/bias*
T0*
_class
loc:@q/bias
_
q/MatMulMatMulh_critic/Reluq/kernel/read*
T0*
transpose_a( *
transpose_b( 
K
	q/BiasAddBiasAddq/MatMulq/bias/read*
data_formatNHWC*
T0

inputsNoOp^a_in^s_in
'
outputsNoOp
^a_out/mul
^q/BiasAdd
@
targetPlaceholder*
dtype0*
shape:ÿÿÿÿÿÿÿÿÿ
U
$mean_squared_error/SquaredDifferenceSquaredDifference	q/BiasAddtarget*
T0
\
/mean_squared_error/assert_broadcastable/weightsConst*
valueB
 *  ?*
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
3mean_squared_error/assert_broadcastable/values/rankConst*
dtype0*
value	B :
K
Cmean_squared_error/assert_broadcastable/static_scalar_check_successNoOp

mean_squared_error/ToFloat/xConstD^mean_squared_error/assert_broadcastable/static_scalar_check_success*
valueB
 *  ?*
dtype0
j
mean_squared_error/MulMul$mean_squared_error/SquaredDifferencemean_squared_error/ToFloat/x*
T0

mean_squared_error/ConstConstD^mean_squared_error/assert_broadcastable/static_scalar_check_success*
dtype0*
valueB"       
u
mean_squared_error/SumSummean_squared_error/Mulmean_squared_error/Const*
T0*

Tidx0*
	keep_dims( 

&mean_squared_error/num_present/Equal/yConstD^mean_squared_error/assert_broadcastable/static_scalar_check_success*
valueB
 *    *
dtype0
|
$mean_squared_error/num_present/EqualEqualmean_squared_error/ToFloat/x&mean_squared_error/num_present/Equal/y*
T0

)mean_squared_error/num_present/zeros_likeConstD^mean_squared_error/assert_broadcastable/static_scalar_check_success*
valueB
 *    *
dtype0

.mean_squared_error/num_present/ones_like/ShapeConstD^mean_squared_error/assert_broadcastable/static_scalar_check_success*
valueB *
dtype0
¡
.mean_squared_error/num_present/ones_like/ConstConstD^mean_squared_error/assert_broadcastable/static_scalar_check_success*
valueB
 *  ?*
dtype0
«
(mean_squared_error/num_present/ones_likeFill.mean_squared_error/num_present/ones_like/Shape.mean_squared_error/num_present/ones_like/Const*
T0*

index_type0
³
%mean_squared_error/num_present/SelectSelect$mean_squared_error/num_present/Equal)mean_squared_error/num_present/zeros_like(mean_squared_error/num_present/ones_like*
T0
Â
Smean_squared_error/num_present/broadcast_weights/assert_broadcastable/weights/shapeConstD^mean_squared_error/assert_broadcastable/static_scalar_check_success*
valueB *
dtype0
Â
Rmean_squared_error/num_present/broadcast_weights/assert_broadcastable/weights/rankConstD^mean_squared_error/assert_broadcastable/static_scalar_check_success*
value	B : *
dtype0
à
Rmean_squared_error/num_present/broadcast_weights/assert_broadcastable/values/shapeShape$mean_squared_error/SquaredDifferenceD^mean_squared_error/assert_broadcastable/static_scalar_check_success*
out_type0*
T0
Á
Qmean_squared_error/num_present/broadcast_weights/assert_broadcastable/values/rankConstD^mean_squared_error/assert_broadcastable/static_scalar_check_success*
value	B :*
dtype0
¯
amean_squared_error/num_present/broadcast_weights/assert_broadcastable/static_scalar_check_successNoOpD^mean_squared_error/assert_broadcastable/static_scalar_check_success
²
@mean_squared_error/num_present/broadcast_weights/ones_like/ShapeShape$mean_squared_error/SquaredDifferenceD^mean_squared_error/assert_broadcastable/static_scalar_check_successb^mean_squared_error/num_present/broadcast_weights/assert_broadcastable/static_scalar_check_success*
T0*
out_type0

@mean_squared_error/num_present/broadcast_weights/ones_like/ConstConstD^mean_squared_error/assert_broadcastable/static_scalar_check_successb^mean_squared_error/num_present/broadcast_weights/assert_broadcastable/static_scalar_check_success*
valueB
 *  ?*
dtype0
á
:mean_squared_error/num_present/broadcast_weights/ones_likeFill@mean_squared_error/num_present/broadcast_weights/ones_like/Shape@mean_squared_error/num_present/broadcast_weights/ones_like/Const*
T0*

index_type0
£
0mean_squared_error/num_present/broadcast_weightsMul%mean_squared_error/num_present/Select:mean_squared_error/num_present/broadcast_weights/ones_like*
T0

$mean_squared_error/num_present/ConstConstD^mean_squared_error/assert_broadcastable/static_scalar_check_success*
valueB"       *
dtype0
£
mean_squared_error/num_presentSum0mean_squared_error/num_present/broadcast_weights$mean_squared_error/num_present/Const*

Tidx0*
	keep_dims( *
T0

mean_squared_error/Const_1ConstD^mean_squared_error/assert_broadcastable/static_scalar_check_success*
valueB *
dtype0
y
mean_squared_error/Sum_1Summean_squared_error/Summean_squared_error/Const_1*
T0*

Tidx0*
	keep_dims( 

mean_squared_error/Greater/yConstD^mean_squared_error/assert_broadcastable/static_scalar_check_success*
valueB
 *    *
dtype0
l
mean_squared_error/GreaterGreatermean_squared_error/num_presentmean_squared_error/Greater/y*
T0

mean_squared_error/Equal/yConstD^mean_squared_error/assert_broadcastable/static_scalar_check_success*
valueB
 *    *
dtype0
f
mean_squared_error/EqualEqualmean_squared_error/num_presentmean_squared_error/Equal/y*
T0

"mean_squared_error/ones_like/ShapeConstD^mean_squared_error/assert_broadcastable/static_scalar_check_success*
valueB *
dtype0

"mean_squared_error/ones_like/ConstConstD^mean_squared_error/assert_broadcastable/static_scalar_check_success*
dtype0*
valueB
 *  ?

mean_squared_error/ones_likeFill"mean_squared_error/ones_like/Shape"mean_squared_error/ones_like/Const*
T0*

index_type0

mean_squared_error/SelectSelectmean_squared_error/Equalmean_squared_error/ones_likemean_squared_error/num_present*
T0
_
mean_squared_error/divRealDivmean_squared_error/Sum_1mean_squared_error/Select*
T0

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
 *  ?*
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
¡
.gradients/mean_squared_error/value_grad/SelectSelectmean_squared_error/Greatergradients/Fill2gradients/mean_squared_error/value_grad/zeros_like*
T0
£
0gradients/mean_squared_error/value_grad/Select_1Selectmean_squared_error/Greater2gradients/mean_squared_error/value_grad/zeros_likegradients/Fill*
T0
¤
8gradients/mean_squared_error/value_grad/tuple/group_depsNoOp/^gradients/mean_squared_error/value_grad/Select1^gradients/mean_squared_error/value_grad/Select_1

@gradients/mean_squared_error/value_grad/tuple/control_dependencyIdentity.gradients/mean_squared_error/value_grad/Select9^gradients/mean_squared_error/value_grad/tuple/group_deps*
T0*A
_class7
53loc:@gradients/mean_squared_error/value_grad/Select

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
¹
;gradients/mean_squared_error/div_grad/BroadcastGradientArgsBroadcastGradientArgs+gradients/mean_squared_error/div_grad/Shape-gradients/mean_squared_error/div_grad/Shape_1*
T0

-gradients/mean_squared_error/div_grad/RealDivRealDiv@gradients/mean_squared_error/value_grad/tuple/control_dependencymean_squared_error/Select*
T0
Â
)gradients/mean_squared_error/div_grad/SumSum-gradients/mean_squared_error/div_grad/RealDiv;gradients/mean_squared_error/div_grad/BroadcastGradientArgs*

Tidx0*
	keep_dims( *
T0
§
-gradients/mean_squared_error/div_grad/ReshapeReshape)gradients/mean_squared_error/div_grad/Sum+gradients/mean_squared_error/div_grad/Shape*
T0*
Tshape0
S
)gradients/mean_squared_error/div_grad/NegNegmean_squared_error/Sum_1*
T0

/gradients/mean_squared_error/div_grad/RealDiv_1RealDiv)gradients/mean_squared_error/div_grad/Negmean_squared_error/Select*
T0

/gradients/mean_squared_error/div_grad/RealDiv_2RealDiv/gradients/mean_squared_error/div_grad/RealDiv_1mean_squared_error/Select*
T0
¬
)gradients/mean_squared_error/div_grad/mulMul@gradients/mean_squared_error/value_grad/tuple/control_dependency/gradients/mean_squared_error/div_grad/RealDiv_2*
T0
Â
+gradients/mean_squared_error/div_grad/Sum_1Sum)gradients/mean_squared_error/div_grad/mul=gradients/mean_squared_error/div_grad/BroadcastGradientArgs:1*

Tidx0*
	keep_dims( *
T0
­
/gradients/mean_squared_error/div_grad/Reshape_1Reshape+gradients/mean_squared_error/div_grad/Sum_1-gradients/mean_squared_error/div_grad/Shape_1*
Tshape0*
T0
 
6gradients/mean_squared_error/div_grad/tuple/group_depsNoOp.^gradients/mean_squared_error/div_grad/Reshape0^gradients/mean_squared_error/div_grad/Reshape_1
ı
>gradients/mean_squared_error/div_grad/tuple/control_dependencyIdentity-gradients/mean_squared_error/div_grad/Reshape7^gradients/mean_squared_error/div_grad/tuple/group_deps*@
_class6
42loc:@gradients/mean_squared_error/div_grad/Reshape*
T0

@gradients/mean_squared_error/div_grad/tuple/control_dependency_1Identity/gradients/mean_squared_error/div_grad/Reshape_17^gradients/mean_squared_error/div_grad/tuple/group_deps*
T0*B
_class8
64loc:@gradients/mean_squared_error/div_grad/Reshape_1
^
5gradients/mean_squared_error/Sum_1_grad/Reshape/shapeConst*
valueB *
dtype0
È
/gradients/mean_squared_error/Sum_1_grad/ReshapeReshape>gradients/mean_squared_error/div_grad/tuple/control_dependency5gradients/mean_squared_error/Sum_1_grad/Reshape/shape*
T0*
Tshape0
V
-gradients/mean_squared_error/Sum_1_grad/ConstConst*
valueB *
dtype0
¯
,gradients/mean_squared_error/Sum_1_grad/TileTile/gradients/mean_squared_error/Sum_1_grad/Reshape-gradients/mean_squared_error/Sum_1_grad/Const*

Tmultiples0*
T0
h
3gradients/mean_squared_error/Sum_grad/Reshape/shapeConst*
valueB"      *
dtype0
²
-gradients/mean_squared_error/Sum_grad/ReshapeReshape,gradients/mean_squared_error/Sum_1_grad/Tile3gradients/mean_squared_error/Sum_grad/Reshape/shape*
T0*
Tshape0
e
+gradients/mean_squared_error/Sum_grad/ShapeShapemean_squared_error/Mul*
T0*
out_type0
©
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
¹
;gradients/mean_squared_error/Mul_grad/BroadcastGradientArgsBroadcastGradientArgs+gradients/mean_squared_error/Mul_grad/Shape-gradients/mean_squared_error/Mul_grad/Shape_1*
T0

)gradients/mean_squared_error/Mul_grad/MulMul*gradients/mean_squared_error/Sum_grad/Tilemean_squared_error/ToFloat/x*
T0
¾
)gradients/mean_squared_error/Mul_grad/SumSum)gradients/mean_squared_error/Mul_grad/Mul;gradients/mean_squared_error/Mul_grad/BroadcastGradientArgs*

Tidx0*
	keep_dims( *
T0
§
-gradients/mean_squared_error/Mul_grad/ReshapeReshape)gradients/mean_squared_error/Mul_grad/Sum+gradients/mean_squared_error/Mul_grad/Shape*
T0*
Tshape0

+gradients/mean_squared_error/Mul_grad/Mul_1Mul$mean_squared_error/SquaredDifference*gradients/mean_squared_error/Sum_grad/Tile*
T0
Ä
+gradients/mean_squared_error/Mul_grad/Sum_1Sum+gradients/mean_squared_error/Mul_grad/Mul_1=gradients/mean_squared_error/Mul_grad/BroadcastGradientArgs:1*

Tidx0*
	keep_dims( *
T0
­
/gradients/mean_squared_error/Mul_grad/Reshape_1Reshape+gradients/mean_squared_error/Mul_grad/Sum_1-gradients/mean_squared_error/Mul_grad/Shape_1*
T0*
Tshape0
 
6gradients/mean_squared_error/Mul_grad/tuple/group_depsNoOp.^gradients/mean_squared_error/Mul_grad/Reshape0^gradients/mean_squared_error/Mul_grad/Reshape_1
ı
>gradients/mean_squared_error/Mul_grad/tuple/control_dependencyIdentity-gradients/mean_squared_error/Mul_grad/Reshape7^gradients/mean_squared_error/Mul_grad/tuple/group_deps*
T0*@
_class6
42loc:@gradients/mean_squared_error/Mul_grad/Reshape

@gradients/mean_squared_error/Mul_grad/tuple/control_dependency_1Identity/gradients/mean_squared_error/Mul_grad/Reshape_17^gradients/mean_squared_error/Mul_grad/tuple/group_deps*
T0*B
_class8
64loc:@gradients/mean_squared_error/Mul_grad/Reshape_1
f
9gradients/mean_squared_error/SquaredDifference_grad/ShapeShape	q/BiasAdd*
T0*
out_type0
e
;gradients/mean_squared_error/SquaredDifference_grad/Shape_1Shapetarget*
T0*
out_type0
ã
Igradients/mean_squared_error/SquaredDifference_grad/BroadcastGradientArgsBroadcastGradientArgs9gradients/mean_squared_error/SquaredDifference_grad/Shape;gradients/mean_squared_error/SquaredDifference_grad/Shape_1*
T0
¨
:gradients/mean_squared_error/SquaredDifference_grad/scalarConst?^gradients/mean_squared_error/Mul_grad/tuple/control_dependency*
valueB
 *   @*
dtype0
Ã
7gradients/mean_squared_error/SquaredDifference_grad/mulMul:gradients/mean_squared_error/SquaredDifference_grad/scalar>gradients/mean_squared_error/Mul_grad/tuple/control_dependency*
T0

7gradients/mean_squared_error/SquaredDifference_grad/subSub	q/BiasAddtarget?^gradients/mean_squared_error/Mul_grad/tuple/control_dependency*
T0
»
9gradients/mean_squared_error/SquaredDifference_grad/mul_1Mul7gradients/mean_squared_error/SquaredDifference_grad/mul7gradients/mean_squared_error/SquaredDifference_grad/sub*
T0
ê
7gradients/mean_squared_error/SquaredDifference_grad/SumSum9gradients/mean_squared_error/SquaredDifference_grad/mul_1Igradients/mean_squared_error/SquaredDifference_grad/BroadcastGradientArgs*

Tidx0*
	keep_dims( *
T0
Ñ
;gradients/mean_squared_error/SquaredDifference_grad/ReshapeReshape7gradients/mean_squared_error/SquaredDifference_grad/Sum9gradients/mean_squared_error/SquaredDifference_grad/Shape*
T0*
Tshape0
î
9gradients/mean_squared_error/SquaredDifference_grad/Sum_1Sum9gradients/mean_squared_error/SquaredDifference_grad/mul_1Kgradients/mean_squared_error/SquaredDifference_grad/BroadcastGradientArgs:1*

Tidx0*
	keep_dims( *
T0
×
=gradients/mean_squared_error/SquaredDifference_grad/Reshape_1Reshape9gradients/mean_squared_error/SquaredDifference_grad/Sum_1;gradients/mean_squared_error/SquaredDifference_grad/Shape_1*
T0*
Tshape0

7gradients/mean_squared_error/SquaredDifference_grad/NegNeg=gradients/mean_squared_error/SquaredDifference_grad/Reshape_1*
T0
Ä
Dgradients/mean_squared_error/SquaredDifference_grad/tuple/group_depsNoOp8^gradients/mean_squared_error/SquaredDifference_grad/Neg<^gradients/mean_squared_error/SquaredDifference_grad/Reshape
µ
Lgradients/mean_squared_error/SquaredDifference_grad/tuple/control_dependencyIdentity;gradients/mean_squared_error/SquaredDifference_grad/ReshapeE^gradients/mean_squared_error/SquaredDifference_grad/tuple/group_deps*N
_classD
B@loc:@gradients/mean_squared_error/SquaredDifference_grad/Reshape*
T0
¯
Ngradients/mean_squared_error/SquaredDifference_grad/tuple/control_dependency_1Identity7gradients/mean_squared_error/SquaredDifference_grad/NegE^gradients/mean_squared_error/SquaredDifference_grad/tuple/group_deps*
T0*J
_class@
><loc:@gradients/mean_squared_error/SquaredDifference_grad/Neg
¡
$gradients/q/BiasAdd_grad/BiasAddGradBiasAddGradLgradients/mean_squared_error/SquaredDifference_grad/tuple/control_dependency*
T0*
data_formatNHWC
§
)gradients/q/BiasAdd_grad/tuple/group_depsNoOpM^gradients/mean_squared_error/SquaredDifference_grad/tuple/control_dependency%^gradients/q/BiasAdd_grad/BiasAddGrad

1gradients/q/BiasAdd_grad/tuple/control_dependencyIdentityLgradients/mean_squared_error/SquaredDifference_grad/tuple/control_dependency*^gradients/q/BiasAdd_grad/tuple/group_deps*
T0*N
_classD
B@loc:@gradients/mean_squared_error/SquaredDifference_grad/Reshape
Ó
3gradients/q/BiasAdd_grad/tuple/control_dependency_1Identity$gradients/q/BiasAdd_grad/BiasAddGrad*^gradients/q/BiasAdd_grad/tuple/group_deps*
T0*7
_class-
+)loc:@gradients/q/BiasAdd_grad/BiasAddGrad

gradients/q/MatMul_grad/MatMulMatMul1gradients/q/BiasAdd_grad/tuple/control_dependencyq/kernel/read*
transpose_b(*
T0*
transpose_a( 

 gradients/q/MatMul_grad/MatMul_1MatMulh_critic/Relu1gradients/q/BiasAdd_grad/tuple/control_dependency*
T0*
transpose_a(*
transpose_b( 
t
(gradients/q/MatMul_grad/tuple/group_depsNoOp^gradients/q/MatMul_grad/MatMul!^gradients/q/MatMul_grad/MatMul_1
Ã
0gradients/q/MatMul_grad/tuple/control_dependencyIdentitygradients/q/MatMul_grad/MatMul)^gradients/q/MatMul_grad/tuple/group_deps*
T0*1
_class'
%#loc:@gradients/q/MatMul_grad/MatMul
É
2gradients/q/MatMul_grad/tuple/control_dependency_1Identity gradients/q/MatMul_grad/MatMul_1)^gradients/q/MatMul_grad/tuple/group_deps*
T0*3
_class)
'%loc:@gradients/q/MatMul_grad/MatMul_1
{
%gradients/h_critic/Relu_grad/ReluGradReluGrad0gradients/q/MatMul_grad/tuple/control_dependencyh_critic/Relu*
T0

+gradients/h_critic/BiasAdd_grad/BiasAddGradBiasAddGrad%gradients/h_critic/Relu_grad/ReluGrad*
T0*
data_formatNHWC

0gradients/h_critic/BiasAdd_grad/tuple/group_depsNoOp,^gradients/h_critic/BiasAdd_grad/BiasAddGrad&^gradients/h_critic/Relu_grad/ReluGrad
á
8gradients/h_critic/BiasAdd_grad/tuple/control_dependencyIdentity%gradients/h_critic/Relu_grad/ReluGrad1^gradients/h_critic/BiasAdd_grad/tuple/group_deps*
T0*8
_class.
,*loc:@gradients/h_critic/Relu_grad/ReluGrad
ï
:gradients/h_critic/BiasAdd_grad/tuple/control_dependency_1Identity+gradients/h_critic/BiasAdd_grad/BiasAddGrad1^gradients/h_critic/BiasAdd_grad/tuple/group_deps*>
_class4
20loc:@gradients/h_critic/BiasAdd_grad/BiasAddGrad*
T0
®
%gradients/h_critic/MatMul_grad/MatMulMatMul8gradients/h_critic/BiasAdd_grad/tuple/control_dependencyh_critic/kernel/read*
transpose_a( *
transpose_b(*
T0
°
'gradients/h_critic/MatMul_grad/MatMul_1MatMulconcatenate_1/concat8gradients/h_critic/BiasAdd_grad/tuple/control_dependency*
T0*
transpose_a(*
transpose_b( 

/gradients/h_critic/MatMul_grad/tuple/group_depsNoOp&^gradients/h_critic/MatMul_grad/MatMul(^gradients/h_critic/MatMul_grad/MatMul_1
ß
7gradients/h_critic/MatMul_grad/tuple/control_dependencyIdentity%gradients/h_critic/MatMul_grad/MatMul0^gradients/h_critic/MatMul_grad/tuple/group_deps*
T0*8
_class.
,*loc:@gradients/h_critic/MatMul_grad/MatMul
å
9gradients/h_critic/MatMul_grad/tuple/control_dependency_1Identity'gradients/h_critic/MatMul_grad/MatMul_10^gradients/h_critic/MatMul_grad/tuple/group_deps*
T0*:
_class0
.,loc:@gradients/h_critic/MatMul_grad/MatMul_1
R
(gradients/concatenate_1/concat_grad/RankConst*
value	B :*
dtype0

'gradients/concatenate_1/concat_grad/modFloorModconcatenate_1/concat/axis(gradients/concatenate_1/concat_grad/Rank*
T0
[
)gradients/concatenate_1/concat_grad/ShapeShapeh_common2/Relu*
T0*
out_type0
l
*gradients/concatenate_1/concat_grad/ShapeNShapeNh_common2/Relua_in*
T0*
out_type0*
N
Ì
0gradients/concatenate_1/concat_grad/ConcatOffsetConcatOffset'gradients/concatenate_1/concat_grad/mod*gradients/concatenate_1/concat_grad/ShapeN,gradients/concatenate_1/concat_grad/ShapeN:1*
N
ß
)gradients/concatenate_1/concat_grad/SliceSlice7gradients/h_critic/MatMul_grad/tuple/control_dependency0gradients/concatenate_1/concat_grad/ConcatOffset*gradients/concatenate_1/concat_grad/ShapeN*
T0*
Index0
å
+gradients/concatenate_1/concat_grad/Slice_1Slice7gradients/h_critic/MatMul_grad/tuple/control_dependency2gradients/concatenate_1/concat_grad/ConcatOffset:1,gradients/concatenate_1/concat_grad/ShapeN:1*
T0*
Index0

4gradients/concatenate_1/concat_grad/tuple/group_depsNoOp*^gradients/concatenate_1/concat_grad/Slice,^gradients/concatenate_1/concat_grad/Slice_1
ñ
<gradients/concatenate_1/concat_grad/tuple/control_dependencyIdentity)gradients/concatenate_1/concat_grad/Slice5^gradients/concatenate_1/concat_grad/tuple/group_deps*
T0*<
_class2
0.loc:@gradients/concatenate_1/concat_grad/Slice
÷
>gradients/concatenate_1/concat_grad/tuple/control_dependency_1Identity+gradients/concatenate_1/concat_grad/Slice_15^gradients/concatenate_1/concat_grad/tuple/group_deps*
T0*>
_class4
20loc:@gradients/concatenate_1/concat_grad/Slice_1

&gradients/h_common2/Relu_grad/ReluGradReluGrad<gradients/concatenate_1/concat_grad/tuple/control_dependencyh_common2/Relu*
T0

,gradients/h_common2/BiasAdd_grad/BiasAddGradBiasAddGrad&gradients/h_common2/Relu_grad/ReluGrad*
T0*
data_formatNHWC

1gradients/h_common2/BiasAdd_grad/tuple/group_depsNoOp-^gradients/h_common2/BiasAdd_grad/BiasAddGrad'^gradients/h_common2/Relu_grad/ReluGrad
å
9gradients/h_common2/BiasAdd_grad/tuple/control_dependencyIdentity&gradients/h_common2/Relu_grad/ReluGrad2^gradients/h_common2/BiasAdd_grad/tuple/group_deps*
T0*9
_class/
-+loc:@gradients/h_common2/Relu_grad/ReluGrad
ó
;gradients/h_common2/BiasAdd_grad/tuple/control_dependency_1Identity,gradients/h_common2/BiasAdd_grad/BiasAddGrad2^gradients/h_common2/BiasAdd_grad/tuple/group_deps*
T0*?
_class5
31loc:@gradients/h_common2/BiasAdd_grad/BiasAddGrad
±
&gradients/h_common2/MatMul_grad/MatMulMatMul9gradients/h_common2/BiasAdd_grad/tuple/control_dependencyh_common2/kernel/read*
transpose_a( *
transpose_b(*
T0
¢
(gradients/h_common2/MatMul_grad/MatMul_1MatMuls_in9gradients/h_common2/BiasAdd_grad/tuple/control_dependency*
T0*
transpose_a(*
transpose_b( 

0gradients/h_common2/MatMul_grad/tuple/group_depsNoOp'^gradients/h_common2/MatMul_grad/MatMul)^gradients/h_common2/MatMul_grad/MatMul_1
ã
8gradients/h_common2/MatMul_grad/tuple/control_dependencyIdentity&gradients/h_common2/MatMul_grad/MatMul1^gradients/h_common2/MatMul_grad/tuple/group_deps*
T0*9
_class/
-+loc:@gradients/h_common2/MatMul_grad/MatMul
é
:gradients/h_common2/MatMul_grad/tuple/control_dependency_1Identity(gradients/h_common2/MatMul_grad/MatMul_11^gradients/h_common2/MatMul_grad/tuple/group_deps*
T0*;
_class1
/-loc:@gradients/h_common2/MatMul_grad/MatMul_1
i
beta1_power/initial_valueConst*
valueB
 *fff?*!
_class
loc:@h_common2/bias*
dtype0
z
beta1_power
VariableV2*!
_class
loc:@h_common2/bias*
dtype0*
	container *
shape: *
shared_name 

beta1_power/AssignAssignbeta1_powerbeta1_power/initial_value*!
_class
loc:@h_common2/bias*
validate_shape(*
use_locking(*
T0
U
beta1_power/readIdentitybeta1_power*
T0*!
_class
loc:@h_common2/bias
i
beta2_power/initial_valueConst*
valueB
 *w¾?*!
_class
loc:@h_common2/bias*
dtype0
z
beta2_power
VariableV2*
shape: *
shared_name *!
_class
loc:@h_common2/bias*
dtype0*
	container 

beta2_power/AssignAssignbeta2_powerbeta2_power/initial_value*
validate_shape(*
use_locking(*
T0*!
_class
loc:@h_common2/bias
U
beta2_power/readIdentitybeta2_power*
T0*!
_class
loc:@h_common2/bias

'h_common2/kernel/Adam/Initializer/zerosConst*
valueB	*    *#
_class
loc:@h_common2/kernel*
dtype0

h_common2/kernel/Adam
VariableV2*
shape:	*
shared_name *#
_class
loc:@h_common2/kernel*
dtype0*
	container 
½
h_common2/kernel/Adam/AssignAssignh_common2/kernel/Adam'h_common2/kernel/Adam/Initializer/zeros*
validate_shape(*
use_locking(*
T0*#
_class
loc:@h_common2/kernel
k
h_common2/kernel/Adam/readIdentityh_common2/kernel/Adam*
T0*#
_class
loc:@h_common2/kernel

)h_common2/kernel/Adam_1/Initializer/zerosConst*
valueB	*    *#
_class
loc:@h_common2/kernel*
dtype0

h_common2/kernel/Adam_1
VariableV2*
shared_name *#
_class
loc:@h_common2/kernel*
dtype0*
	container *
shape:	
Ã
h_common2/kernel/Adam_1/AssignAssignh_common2/kernel/Adam_1)h_common2/kernel/Adam_1/Initializer/zeros*
T0*#
_class
loc:@h_common2/kernel*
validate_shape(*
use_locking(
o
h_common2/kernel/Adam_1/readIdentityh_common2/kernel/Adam_1*#
_class
loc:@h_common2/kernel*
T0
z
%h_common2/bias/Adam/Initializer/zerosConst*
valueB*    *!
_class
loc:@h_common2/bias*
dtype0

h_common2/bias/Adam
VariableV2*
shape:*
shared_name *!
_class
loc:@h_common2/bias*
dtype0*
	container 
µ
h_common2/bias/Adam/AssignAssignh_common2/bias/Adam%h_common2/bias/Adam/Initializer/zeros*
validate_shape(*
use_locking(*
T0*!
_class
loc:@h_common2/bias
e
h_common2/bias/Adam/readIdentityh_common2/bias/Adam*
T0*!
_class
loc:@h_common2/bias
|
'h_common2/bias/Adam_1/Initializer/zerosConst*
dtype0*
valueB*    *!
_class
loc:@h_common2/bias

h_common2/bias/Adam_1
VariableV2*
shape:*
shared_name *!
_class
loc:@h_common2/bias*
dtype0*
	container 
»
h_common2/bias/Adam_1/AssignAssignh_common2/bias/Adam_1'h_common2/bias/Adam_1/Initializer/zeros*
use_locking(*
T0*!
_class
loc:@h_common2/bias*
validate_shape(
i
h_common2/bias/Adam_1/readIdentityh_common2/bias/Adam_1*
T0*!
_class
loc:@h_common2/bias

6h_critic/kernel/Adam/Initializer/zeros/shape_as_tensorConst*
valueB"  ,  *"
_class
loc:@h_critic/kernel*
dtype0
}
,h_critic/kernel/Adam/Initializer/zeros/ConstConst*
valueB
 *    *"
_class
loc:@h_critic/kernel*
dtype0
Ó
&h_critic/kernel/Adam/Initializer/zerosFill6h_critic/kernel/Adam/Initializer/zeros/shape_as_tensor,h_critic/kernel/Adam/Initializer/zeros/Const*
T0*

index_type0*"
_class
loc:@h_critic/kernel

h_critic/kernel/Adam
VariableV2*
shape:
¬*
shared_name *"
_class
loc:@h_critic/kernel*
dtype0*
	container 
¹
h_critic/kernel/Adam/AssignAssignh_critic/kernel/Adam&h_critic/kernel/Adam/Initializer/zeros*
validate_shape(*
use_locking(*
T0*"
_class
loc:@h_critic/kernel
h
h_critic/kernel/Adam/readIdentityh_critic/kernel/Adam*
T0*"
_class
loc:@h_critic/kernel

8h_critic/kernel/Adam_1/Initializer/zeros/shape_as_tensorConst*
valueB"  ,  *"
_class
loc:@h_critic/kernel*
dtype0

.h_critic/kernel/Adam_1/Initializer/zeros/ConstConst*
valueB
 *    *"
_class
loc:@h_critic/kernel*
dtype0
Ù
(h_critic/kernel/Adam_1/Initializer/zerosFill8h_critic/kernel/Adam_1/Initializer/zeros/shape_as_tensor.h_critic/kernel/Adam_1/Initializer/zeros/Const*
T0*

index_type0*"
_class
loc:@h_critic/kernel

h_critic/kernel/Adam_1
VariableV2*
	container *
shape:
¬*
shared_name *"
_class
loc:@h_critic/kernel*
dtype0
¿
h_critic/kernel/Adam_1/AssignAssignh_critic/kernel/Adam_1(h_critic/kernel/Adam_1/Initializer/zeros*
T0*"
_class
loc:@h_critic/kernel*
validate_shape(*
use_locking(
l
h_critic/kernel/Adam_1/readIdentityh_critic/kernel/Adam_1*
T0*"
_class
loc:@h_critic/kernel
x
$h_critic/bias/Adam/Initializer/zerosConst*
valueB¬*    * 
_class
loc:@h_critic/bias*
dtype0

h_critic/bias/Adam
VariableV2*
dtype0*
	container *
shape:¬*
shared_name * 
_class
loc:@h_critic/bias
±
h_critic/bias/Adam/AssignAssignh_critic/bias/Adam$h_critic/bias/Adam/Initializer/zeros*
use_locking(*
T0* 
_class
loc:@h_critic/bias*
validate_shape(
b
h_critic/bias/Adam/readIdentityh_critic/bias/Adam*
T0* 
_class
loc:@h_critic/bias
z
&h_critic/bias/Adam_1/Initializer/zerosConst*
valueB¬*    * 
_class
loc:@h_critic/bias*
dtype0

h_critic/bias/Adam_1
VariableV2*
shared_name * 
_class
loc:@h_critic/bias*
dtype0*
	container *
shape:¬
·
h_critic/bias/Adam_1/AssignAssignh_critic/bias/Adam_1&h_critic/bias/Adam_1/Initializer/zeros*
use_locking(*
T0* 
_class
loc:@h_critic/bias*
validate_shape(
f
h_critic/bias/Adam_1/readIdentityh_critic/bias/Adam_1*
T0* 
_class
loc:@h_critic/bias
r
q/kernel/Adam/Initializer/zerosConst*
valueB	¬*    *
_class
loc:@q/kernel*
dtype0

q/kernel/Adam
VariableV2*
_class
loc:@q/kernel*
dtype0*
	container *
shape:	¬*
shared_name 

q/kernel/Adam/AssignAssignq/kernel/Adamq/kernel/Adam/Initializer/zeros*
_class
loc:@q/kernel*
validate_shape(*
use_locking(*
T0
S
q/kernel/Adam/readIdentityq/kernel/Adam*
T0*
_class
loc:@q/kernel
t
!q/kernel/Adam_1/Initializer/zerosConst*
valueB	¬*    *
_class
loc:@q/kernel*
dtype0

q/kernel/Adam_1
VariableV2*
_class
loc:@q/kernel*
dtype0*
	container *
shape:	¬*
shared_name 
£
q/kernel/Adam_1/AssignAssignq/kernel/Adam_1!q/kernel/Adam_1/Initializer/zeros*
_class
loc:@q/kernel*
validate_shape(*
use_locking(*
T0
W
q/kernel/Adam_1/readIdentityq/kernel/Adam_1*
T0*
_class
loc:@q/kernel
i
q/bias/Adam/Initializer/zerosConst*
valueB*    *
_class
loc:@q/bias*
dtype0
v
q/bias/Adam
VariableV2*
_class
loc:@q/bias*
dtype0*
	container *
shape:*
shared_name 

q/bias/Adam/AssignAssignq/bias/Adamq/bias/Adam/Initializer/zeros*
T0*
_class
loc:@q/bias*
validate_shape(*
use_locking(
M
q/bias/Adam/readIdentityq/bias/Adam*
T0*
_class
loc:@q/bias
k
q/bias/Adam_1/Initializer/zerosConst*
valueB*    *
_class
loc:@q/bias*
dtype0
x
q/bias/Adam_1
VariableV2*
_class
loc:@q/bias*
dtype0*
	container *
shape:*
shared_name 

q/bias/Adam_1/AssignAssignq/bias/Adam_1q/bias/Adam_1/Initializer/zeros*
use_locking(*
T0*
_class
loc:@q/bias*
validate_shape(
Q
q/bias/Adam_1/readIdentityq/bias/Adam_1*
T0*
_class
loc:@q/bias
A
update/learning_rateConst*
valueB
 *o:*
dtype0
9
update/beta1Const*
valueB
 *fff?*
dtype0
9
update/beta2Const*
valueB
 *w¾?*
dtype0
;
update/epsilonConst*
valueB
 *wÌ+2*
dtype0
î
(update/update_h_common2/kernel/ApplyAdam	ApplyAdamh_common2/kernelh_common2/kernel/Adamh_common2/kernel/Adam_1beta1_power/readbeta2_power/readupdate/learning_rateupdate/beta1update/beta2update/epsilon:gradients/h_common2/MatMul_grad/tuple/control_dependency_1*#
_class
loc:@h_common2/kernel*
use_nesterov( *
use_locking( *
T0
å
&update/update_h_common2/bias/ApplyAdam	ApplyAdamh_common2/biash_common2/bias/Adamh_common2/bias/Adam_1beta1_power/readbeta2_power/readupdate/learning_rateupdate/beta1update/beta2update/epsilon;gradients/h_common2/BiasAdd_grad/tuple/control_dependency_1*
use_nesterov( *
use_locking( *
T0*!
_class
loc:@h_common2/bias
è
'update/update_h_critic/kernel/ApplyAdam	ApplyAdamh_critic/kernelh_critic/kernel/Adamh_critic/kernel/Adam_1beta1_power/readbeta2_power/readupdate/learning_rateupdate/beta1update/beta2update/epsilon9gradients/h_critic/MatMul_grad/tuple/control_dependency_1*
use_locking( *
T0*"
_class
loc:@h_critic/kernel*
use_nesterov( 
ß
%update/update_h_critic/bias/ApplyAdam	ApplyAdamh_critic/biash_critic/bias/Adamh_critic/bias/Adam_1beta1_power/readbeta2_power/readupdate/learning_rateupdate/beta1update/beta2update/epsilon:gradients/h_critic/BiasAdd_grad/tuple/control_dependency_1*
T0* 
_class
loc:@h_critic/bias*
use_nesterov( *
use_locking( 
¾
 update/update_q/kernel/ApplyAdam	ApplyAdamq/kernelq/kernel/Adamq/kernel/Adam_1beta1_power/readbeta2_power/readupdate/learning_rateupdate/beta1update/beta2update/epsilon2gradients/q/MatMul_grad/tuple/control_dependency_1*
use_locking( *
T0*
_class
loc:@q/kernel*
use_nesterov( 
µ
update/update_q/bias/ApplyAdam	ApplyAdamq/biasq/bias/Adamq/bias/Adam_1beta1_power/readbeta2_power/readupdate/learning_rateupdate/beta1update/beta2update/epsilon3gradients/q/BiasAdd_grad/tuple/control_dependency_1*
use_nesterov( *
use_locking( *
T0*
_class
loc:@q/bias
Ç

update/mulMulbeta1_power/readupdate/beta1'^update/update_h_common2/bias/ApplyAdam)^update/update_h_common2/kernel/ApplyAdam&^update/update_h_critic/bias/ApplyAdam(^update/update_h_critic/kernel/ApplyAdam^update/update_q/bias/ApplyAdam!^update/update_q/kernel/ApplyAdam*
T0*!
_class
loc:@h_common2/bias

update/AssignAssignbeta1_power
update/mul*!
_class
loc:@h_common2/bias*
validate_shape(*
use_locking( *
T0
É
update/mul_1Mulbeta2_power/readupdate/beta2'^update/update_h_common2/bias/ApplyAdam)^update/update_h_common2/kernel/ApplyAdam&^update/update_h_critic/bias/ApplyAdam(^update/update_h_critic/kernel/ApplyAdam^update/update_q/bias/ApplyAdam!^update/update_q/kernel/ApplyAdam*
T0*!
_class
loc:@h_common2/bias

update/Assign_1Assignbeta2_powerupdate/mul_1*
use_locking( *
T0*!
_class
loc:@h_common2/bias*
validate_shape(

updateNoOp^update/Assign^update/Assign_1'^update/update_h_common2/bias/ApplyAdam)^update/update_h_common2/kernel/ApplyAdam&^update/update_h_critic/bias/ApplyAdam(^update/update_h_critic/kernel/ApplyAdam^update/update_q/bias/ApplyAdam!^update/update_q/kernel/ApplyAdam
8
dq_da/ShapeShape	q/BiasAdd*
out_type0*
T0
<
dq_da/grad_ys_0Const*
valueB
 *  ?*
dtype0
K

dq_da/FillFilldq_da/Shapedq_da/grad_ys_0*
T0*

index_type0
[
 dq_da/q/BiasAdd_grad/BiasAddGradBiasAddGrad
dq_da/Fill*
T0*
data_formatNHWC
n
dq_da/q/MatMul_grad/MatMulMatMul
dq_da/Fillq/kernel/read*
transpose_b(*
T0*
transpose_a( 
p
dq_da/q/MatMul_grad/MatMul_1MatMulh_critic/Relu
dq_da/Fill*
T0*
transpose_a(*
transpose_b( 
a
!dq_da/h_critic/Relu_grad/ReluGradReluGraddq_da/q/MatMul_grad/MatMulh_critic/Relu*
T0
y
'dq_da/h_critic/BiasAdd_grad/BiasAddGradBiasAddGrad!dq_da/h_critic/Relu_grad/ReluGrad*
T0*
data_formatNHWC

!dq_da/h_critic/MatMul_grad/MatMulMatMul!dq_da/h_critic/Relu_grad/ReluGradh_critic/kernel/read*
T0*
transpose_a( *
transpose_b(

#dq_da/h_critic/MatMul_grad/MatMul_1MatMulconcatenate_1/concat!dq_da/h_critic/Relu_grad/ReluGrad*
transpose_b( *
T0*
transpose_a(
N
$dq_da/concatenate_1/concat_grad/RankConst*
value	B :*
dtype0
y
#dq_da/concatenate_1/concat_grad/modFloorModconcatenate_1/concat/axis$dq_da/concatenate_1/concat_grad/Rank*
T0
W
%dq_da/concatenate_1/concat_grad/ShapeShapeh_common2/Relu*
T0*
out_type0
h
&dq_da/concatenate_1/concat_grad/ShapeNShapeNh_common2/Relua_in*
T0*
out_type0*
N
¼
,dq_da/concatenate_1/concat_grad/ConcatOffsetConcatOffset#dq_da/concatenate_1/concat_grad/mod&dq_da/concatenate_1/concat_grad/ShapeN(dq_da/concatenate_1/concat_grad/ShapeN:1*
N
½
%dq_da/concatenate_1/concat_grad/SliceSlice!dq_da/h_critic/MatMul_grad/MatMul,dq_da/concatenate_1/concat_grad/ConcatOffset&dq_da/concatenate_1/concat_grad/ShapeN*
T0*
Index0
Ã
'dq_da/concatenate_1/concat_grad/Slice_1Slice!dq_da/h_critic/MatMul_grad/MatMul.dq_da/concatenate_1/concat_grad/ConcatOffset:1(dq_da/concatenate_1/concat_grad/ShapeN:1*
T0*
Index0
<
NegNeg'dq_da/concatenate_1/concat_grad/Slice_1*
T0
-
dq_dtheta/grad_ys_0IdentityNeg*
T0
G
dq_dtheta/a_out/mul_grad/ShapeConst*
valueB *
dtype0
N
 dq_dtheta/a_out/mul_grad/Shape_1Shape
a_raw/Tanh*
T0*
out_type0

.dq_dtheta/a_out/mul_grad/BroadcastGradientArgsBroadcastGradientArgsdq_dtheta/a_out/mul_grad/Shape dq_dtheta/a_out/mul_grad/Shape_1*
T0
M
dq_dtheta/a_out/mul_grad/MulMuldq_dtheta/grad_ys_0
a_raw/Tanh*
T0

dq_dtheta/a_out/mul_grad/SumSumdq_dtheta/a_out/mul_grad/Mul.dq_dtheta/a_out/mul_grad/BroadcastGradientArgs*

Tidx0*
	keep_dims( *
T0

 dq_dtheta/a_out/mul_grad/ReshapeReshapedq_dtheta/a_out/mul_grad/Sumdq_dtheta/a_out/mul_grad/Shape*
T0*
Tshape0
P
dq_dtheta/a_out/mul_grad/Mul_1Mula_out/mul/xdq_dtheta/grad_ys_0*
T0

dq_dtheta/a_out/mul_grad/Sum_1Sumdq_dtheta/a_out/mul_grad/Mul_10dq_dtheta/a_out/mul_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 

"dq_dtheta/a_out/mul_grad/Reshape_1Reshapedq_dtheta/a_out/mul_grad/Sum_1 dq_dtheta/a_out/mul_grad/Shape_1*
T0*
Tshape0
g
"dq_dtheta/a_raw/Tanh_grad/TanhGradTanhGrad
a_raw/Tanh"dq_dtheta/a_out/mul_grad/Reshape_1*
T0
{
(dq_dtheta/a_raw/BiasAdd_grad/BiasAddGradBiasAddGrad"dq_dtheta/a_raw/Tanh_grad/TanhGrad*
T0*
data_formatNHWC

"dq_dtheta/a_raw/MatMul_grad/MatMulMatMul"dq_dtheta/a_raw/Tanh_grad/TanhGrada_raw/kernel/read*
transpose_b(*
T0*
transpose_a( 

$dq_dtheta/a_raw/MatMul_grad/MatMul_1MatMulh_actor/Relu"dq_dtheta/a_raw/Tanh_grad/TanhGrad*
transpose_b( *
T0*
transpose_a(
k
$dq_dtheta/h_actor/Relu_grad/ReluGradReluGrad"dq_dtheta/a_raw/MatMul_grad/MatMulh_actor/Relu*
T0

*dq_dtheta/h_actor/BiasAdd_grad/BiasAddGradBiasAddGrad$dq_dtheta/h_actor/Relu_grad/ReluGrad*
T0*
data_formatNHWC

$dq_dtheta/h_actor/MatMul_grad/MatMulMatMul$dq_dtheta/h_actor/Relu_grad/ReluGradh_actor/kernel/read*
transpose_a( *
transpose_b(*
T0

&dq_dtheta/h_actor/MatMul_grad/MatMul_1MatMulh_common/Relu$dq_dtheta/h_actor/Relu_grad/ReluGrad*
T0*
transpose_a(*
transpose_b( 
o
%dq_dtheta/h_common/Relu_grad/ReluGradReluGrad$dq_dtheta/h_actor/MatMul_grad/MatMulh_common/Relu*
T0

+dq_dtheta/h_common/BiasAdd_grad/BiasAddGradBiasAddGrad%dq_dtheta/h_common/Relu_grad/ReluGrad*
T0*
data_formatNHWC

%dq_dtheta/h_common/MatMul_grad/MatMulMatMul%dq_dtheta/h_common/Relu_grad/ReluGradh_common/kernel/read*
T0*
transpose_a( *
transpose_b(

'dq_dtheta/h_common/MatMul_grad/MatMul_1MatMuls_in%dq_dtheta/h_common/Relu_grad/ReluGrad*
T0*
transpose_a(*
transpose_b( 
g
beta1_power_1/initial_valueConst*
valueB
 *fff?*
_class
loc:@a_raw/bias*
dtype0
x
beta1_power_1
VariableV2*
	container *
shape: *
shared_name *
_class
loc:@a_raw/bias*
dtype0

beta1_power_1/AssignAssignbeta1_power_1beta1_power_1/initial_value*
validate_shape(*
use_locking(*
T0*
_class
loc:@a_raw/bias
U
beta1_power_1/readIdentitybeta1_power_1*
T0*
_class
loc:@a_raw/bias
g
beta2_power_1/initial_valueConst*
valueB
 *w¾?*
_class
loc:@a_raw/bias*
dtype0
x
beta2_power_1
VariableV2*
shared_name *
_class
loc:@a_raw/bias*
dtype0*
	container *
shape: 

beta2_power_1/AssignAssignbeta2_power_1beta2_power_1/initial_value*
T0*
_class
loc:@a_raw/bias*
validate_shape(*
use_locking(
U
beta2_power_1/readIdentitybeta2_power_1*
T0*
_class
loc:@a_raw/bias

&h_common/kernel/Adam/Initializer/zerosConst*
valueB	*    *"
_class
loc:@h_common/kernel*
dtype0

h_common/kernel/Adam
VariableV2*
shared_name *"
_class
loc:@h_common/kernel*
dtype0*
	container *
shape:	
¹
h_common/kernel/Adam/AssignAssignh_common/kernel/Adam&h_common/kernel/Adam/Initializer/zeros*
use_locking(*
T0*"
_class
loc:@h_common/kernel*
validate_shape(
h
h_common/kernel/Adam/readIdentityh_common/kernel/Adam*
T0*"
_class
loc:@h_common/kernel

(h_common/kernel/Adam_1/Initializer/zerosConst*
valueB	*    *"
_class
loc:@h_common/kernel*
dtype0

h_common/kernel/Adam_1
VariableV2*
shape:	*
shared_name *"
_class
loc:@h_common/kernel*
dtype0*
	container 
¿
h_common/kernel/Adam_1/AssignAssignh_common/kernel/Adam_1(h_common/kernel/Adam_1/Initializer/zeros*
use_locking(*
T0*"
_class
loc:@h_common/kernel*
validate_shape(
l
h_common/kernel/Adam_1/readIdentityh_common/kernel/Adam_1*
T0*"
_class
loc:@h_common/kernel
x
$h_common/bias/Adam/Initializer/zerosConst*
valueB*    * 
_class
loc:@h_common/bias*
dtype0

h_common/bias/Adam
VariableV2* 
_class
loc:@h_common/bias*
dtype0*
	container *
shape:*
shared_name 
±
h_common/bias/Adam/AssignAssignh_common/bias/Adam$h_common/bias/Adam/Initializer/zeros*
validate_shape(*
use_locking(*
T0* 
_class
loc:@h_common/bias
b
h_common/bias/Adam/readIdentityh_common/bias/Adam*
T0* 
_class
loc:@h_common/bias
z
&h_common/bias/Adam_1/Initializer/zerosConst*
dtype0*
valueB*    * 
_class
loc:@h_common/bias

h_common/bias/Adam_1
VariableV2*
shape:*
shared_name * 
_class
loc:@h_common/bias*
dtype0*
	container 
·
h_common/bias/Adam_1/AssignAssignh_common/bias/Adam_1&h_common/bias/Adam_1/Initializer/zeros*
validate_shape(*
use_locking(*
T0* 
_class
loc:@h_common/bias
f
h_common/bias/Adam_1/readIdentityh_common/bias/Adam_1*
T0* 
_class
loc:@h_common/bias

5h_actor/kernel/Adam/Initializer/zeros/shape_as_tensorConst*
valueB"  ,  *!
_class
loc:@h_actor/kernel*
dtype0
{
+h_actor/kernel/Adam/Initializer/zeros/ConstConst*
valueB
 *    *!
_class
loc:@h_actor/kernel*
dtype0
Ï
%h_actor/kernel/Adam/Initializer/zerosFill5h_actor/kernel/Adam/Initializer/zeros/shape_as_tensor+h_actor/kernel/Adam/Initializer/zeros/Const*
T0*

index_type0*!
_class
loc:@h_actor/kernel

h_actor/kernel/Adam
VariableV2*
shape:
¬*
shared_name *!
_class
loc:@h_actor/kernel*
dtype0*
	container 
µ
h_actor/kernel/Adam/AssignAssignh_actor/kernel/Adam%h_actor/kernel/Adam/Initializer/zeros*!
_class
loc:@h_actor/kernel*
validate_shape(*
use_locking(*
T0
e
h_actor/kernel/Adam/readIdentityh_actor/kernel/Adam*
T0*!
_class
loc:@h_actor/kernel

7h_actor/kernel/Adam_1/Initializer/zeros/shape_as_tensorConst*
valueB"  ,  *!
_class
loc:@h_actor/kernel*
dtype0
}
-h_actor/kernel/Adam_1/Initializer/zeros/ConstConst*
valueB
 *    *!
_class
loc:@h_actor/kernel*
dtype0
Õ
'h_actor/kernel/Adam_1/Initializer/zerosFill7h_actor/kernel/Adam_1/Initializer/zeros/shape_as_tensor-h_actor/kernel/Adam_1/Initializer/zeros/Const*
T0*

index_type0*!
_class
loc:@h_actor/kernel

h_actor/kernel/Adam_1
VariableV2*
shared_name *!
_class
loc:@h_actor/kernel*
dtype0*
	container *
shape:
¬
»
h_actor/kernel/Adam_1/AssignAssignh_actor/kernel/Adam_1'h_actor/kernel/Adam_1/Initializer/zeros*
use_locking(*
T0*!
_class
loc:@h_actor/kernel*
validate_shape(
i
h_actor/kernel/Adam_1/readIdentityh_actor/kernel/Adam_1*
T0*!
_class
loc:@h_actor/kernel
v
#h_actor/bias/Adam/Initializer/zerosConst*
valueB¬*    *
_class
loc:@h_actor/bias*
dtype0

h_actor/bias/Adam
VariableV2*
_class
loc:@h_actor/bias*
dtype0*
	container *
shape:¬*
shared_name 
­
h_actor/bias/Adam/AssignAssignh_actor/bias/Adam#h_actor/bias/Adam/Initializer/zeros*
T0*
_class
loc:@h_actor/bias*
validate_shape(*
use_locking(
_
h_actor/bias/Adam/readIdentityh_actor/bias/Adam*
T0*
_class
loc:@h_actor/bias
x
%h_actor/bias/Adam_1/Initializer/zerosConst*
valueB¬*    *
_class
loc:@h_actor/bias*
dtype0

h_actor/bias/Adam_1
VariableV2*
shared_name *
_class
loc:@h_actor/bias*
dtype0*
	container *
shape:¬
³
h_actor/bias/Adam_1/AssignAssignh_actor/bias/Adam_1%h_actor/bias/Adam_1/Initializer/zeros*
T0*
_class
loc:@h_actor/bias*
validate_shape(*
use_locking(
c
h_actor/bias/Adam_1/readIdentityh_actor/bias/Adam_1*
T0*
_class
loc:@h_actor/bias
z
#a_raw/kernel/Adam/Initializer/zerosConst*
valueB	¬*    *
_class
loc:@a_raw/kernel*
dtype0

a_raw/kernel/Adam
VariableV2*
_class
loc:@a_raw/kernel*
dtype0*
	container *
shape:	¬*
shared_name 
­
a_raw/kernel/Adam/AssignAssigna_raw/kernel/Adam#a_raw/kernel/Adam/Initializer/zeros*
use_locking(*
T0*
_class
loc:@a_raw/kernel*
validate_shape(
_
a_raw/kernel/Adam/readIdentitya_raw/kernel/Adam*
T0*
_class
loc:@a_raw/kernel
|
%a_raw/kernel/Adam_1/Initializer/zerosConst*
valueB	¬*    *
_class
loc:@a_raw/kernel*
dtype0

a_raw/kernel/Adam_1
VariableV2*
shape:	¬*
shared_name *
_class
loc:@a_raw/kernel*
dtype0*
	container 
³
a_raw/kernel/Adam_1/AssignAssigna_raw/kernel/Adam_1%a_raw/kernel/Adam_1/Initializer/zeros*
T0*
_class
loc:@a_raw/kernel*
validate_shape(*
use_locking(
c
a_raw/kernel/Adam_1/readIdentitya_raw/kernel/Adam_1*
_class
loc:@a_raw/kernel*
T0
q
!a_raw/bias/Adam/Initializer/zerosConst*
valueB*    *
_class
loc:@a_raw/bias*
dtype0
~
a_raw/bias/Adam
VariableV2*
_class
loc:@a_raw/bias*
dtype0*
	container *
shape:*
shared_name 
¥
a_raw/bias/Adam/AssignAssigna_raw/bias/Adam!a_raw/bias/Adam/Initializer/zeros*
use_locking(*
T0*
_class
loc:@a_raw/bias*
validate_shape(
Y
a_raw/bias/Adam/readIdentitya_raw/bias/Adam*
T0*
_class
loc:@a_raw/bias
s
#a_raw/bias/Adam_1/Initializer/zerosConst*
dtype0*
valueB*    *
_class
loc:@a_raw/bias

a_raw/bias/Adam_1
VariableV2*
	container *
shape:*
shared_name *
_class
loc:@a_raw/bias*
dtype0
«
a_raw/bias/Adam_1/AssignAssigna_raw/bias/Adam_1#a_raw/bias/Adam_1/Initializer/zeros*
use_locking(*
T0*
_class
loc:@a_raw/bias*
validate_shape(
]
a_raw/bias/Adam_1/readIdentitya_raw/bias/Adam_1*
T0*
_class
loc:@a_raw/bias
C
a_update/learning_rateConst*
valueB
 *·Ñ8*
dtype0
;
a_update/beta1Const*
valueB
 *fff?*
dtype0
;
a_update/beta2Const*
valueB
 *w¾?*
dtype0
=
a_update/epsilonConst*
valueB
 *wÌ+2*
dtype0
ä
)a_update/update_h_common/kernel/ApplyAdam	ApplyAdamh_common/kernelh_common/kernel/Adamh_common/kernel/Adam_1beta1_power_1/readbeta2_power_1/reada_update/learning_ratea_update/beta1a_update/beta2a_update/epsilon'dq_dtheta/h_common/MatMul_grad/MatMul_1*
use_locking( *
T0*"
_class
loc:@h_common/kernel*
use_nesterov( 
Ş
'a_update/update_h_common/bias/ApplyAdam	ApplyAdamh_common/biash_common/bias/Adamh_common/bias/Adam_1beta1_power_1/readbeta2_power_1/reada_update/learning_ratea_update/beta1a_update/beta2a_update/epsilon+dq_dtheta/h_common/BiasAdd_grad/BiasAddGrad*
use_locking( *
T0* 
_class
loc:@h_common/bias*
use_nesterov( 
Ş
(a_update/update_h_actor/kernel/ApplyAdam	ApplyAdamh_actor/kernelh_actor/kernel/Adamh_actor/kernel/Adam_1beta1_power_1/readbeta2_power_1/reada_update/learning_ratea_update/beta1a_update/beta2a_update/epsilon&dq_dtheta/h_actor/MatMul_grad/MatMul_1*
T0*!
_class
loc:@h_actor/kernel*
use_nesterov( *
use_locking( 
Ø
&a_update/update_h_actor/bias/ApplyAdam	ApplyAdamh_actor/biash_actor/bias/Adamh_actor/bias/Adam_1beta1_power_1/readbeta2_power_1/reada_update/learning_ratea_update/beta1a_update/beta2a_update/epsilon*dq_dtheta/h_actor/BiasAdd_grad/BiasAddGrad*
use_nesterov( *
use_locking( *
T0*
_class
loc:@h_actor/bias
Ò
&a_update/update_a_raw/kernel/ApplyAdam	ApplyAdama_raw/kernela_raw/kernel/Adama_raw/kernel/Adam_1beta1_power_1/readbeta2_power_1/reada_update/learning_ratea_update/beta1a_update/beta2a_update/epsilon$dq_dtheta/a_raw/MatMul_grad/MatMul_1*
use_locking( *
T0*
_class
loc:@a_raw/kernel*
use_nesterov( 
Ì
$a_update/update_a_raw/bias/ApplyAdam	ApplyAdam
a_raw/biasa_raw/bias/Adama_raw/bias/Adam_1beta1_power_1/readbeta2_power_1/reada_update/learning_ratea_update/beta1a_update/beta2a_update/epsilon(dq_dtheta/a_raw/BiasAdd_grad/BiasAddGrad*
_class
loc:@a_raw/bias*
use_nesterov( *
use_locking( *
T0
Ù
a_update/mulMulbeta1_power_1/reada_update/beta1%^a_update/update_a_raw/bias/ApplyAdam'^a_update/update_a_raw/kernel/ApplyAdam'^a_update/update_h_actor/bias/ApplyAdam)^a_update/update_h_actor/kernel/ApplyAdam(^a_update/update_h_common/bias/ApplyAdam*^a_update/update_h_common/kernel/ApplyAdam*
T0*
_class
loc:@a_raw/bias

a_update/AssignAssignbeta1_power_1a_update/mul*
use_locking( *
T0*
_class
loc:@a_raw/bias*
validate_shape(
Û
a_update/mul_1Mulbeta2_power_1/reada_update/beta2%^a_update/update_a_raw/bias/ApplyAdam'^a_update/update_a_raw/kernel/ApplyAdam'^a_update/update_h_actor/bias/ApplyAdam)^a_update/update_h_actor/kernel/ApplyAdam(^a_update/update_h_common/bias/ApplyAdam*^a_update/update_h_common/kernel/ApplyAdam*
_class
loc:@a_raw/bias*
T0

a_update/Assign_1Assignbeta2_power_1a_update/mul_1*
use_locking( *
T0*
_class
loc:@a_raw/bias*
validate_shape(
°
a_updateNoOp^a_update/Assign^a_update/Assign_1%^a_update/update_a_raw/bias/ApplyAdam'^a_update/update_a_raw/kernel/ApplyAdam'^a_update/update_h_actor/bias/ApplyAdam)^a_update/update_h_actor/kernel/ApplyAdam(^a_update/update_h_common/bias/ApplyAdam*^a_update/update_h_common/kernel/ApplyAdam
=
PlaceholderPlaceholder*
dtype0*
shape:	

AssignAssignh_common/kernelPlaceholder*
validate_shape(*
use_locking(*
T0*"
_class
loc:@h_common/kernel
;
Placeholder_1Placeholder*
shape:*
dtype0

Assign_1Assignh_common/biasPlaceholder_1*
validate_shape(*
use_locking(*
T0* 
_class
loc:@h_common/bias
@
Placeholder_2Placeholder*
shape:
¬*
dtype0

Assign_2Assignh_actor/kernelPlaceholder_2*
T0*!
_class
loc:@h_actor/kernel*
validate_shape(*
use_locking(
;
Placeholder_3Placeholder*
shape:¬*
dtype0

Assign_3Assignh_actor/biasPlaceholder_3*
validate_shape(*
use_locking(*
T0*
_class
loc:@h_actor/bias
?
Placeholder_4Placeholder*
shape:	¬*
dtype0

Assign_4Assigna_raw/kernelPlaceholder_4*
use_locking(*
T0*
_class
loc:@a_raw/kernel*
validate_shape(
:
Placeholder_5Placeholder*
dtype0*
shape:
~
Assign_5Assign
a_raw/biasPlaceholder_5*
validate_shape(*
use_locking(*
T0*
_class
loc:@a_raw/bias
?
Placeholder_6Placeholder*
dtype0*
shape:	

Assign_6Assignh_common2/kernelPlaceholder_6*
use_locking(*
T0*#
_class
loc:@h_common2/kernel*
validate_shape(
;
Placeholder_7Placeholder*
shape:*
dtype0

Assign_7Assignh_common2/biasPlaceholder_7*
T0*!
_class
loc:@h_common2/bias*
validate_shape(*
use_locking(
@
Placeholder_8Placeholder*
dtype0*
shape:
¬

Assign_8Assignh_critic/kernelPlaceholder_8*
use_locking(*
T0*"
_class
loc:@h_critic/kernel*
validate_shape(
;
Placeholder_9Placeholder*
dtype0*
shape:¬

Assign_9Assignh_critic/biasPlaceholder_9*
use_locking(*
T0* 
_class
loc:@h_critic/bias*
validate_shape(
@
Placeholder_10Placeholder*
shape:	¬*
dtype0
|
	Assign_10Assignq/kernelPlaceholder_10*
use_locking(*
T0*
_class
loc:@q/kernel*
validate_shape(
;
Placeholder_11Placeholder*
shape:*
dtype0
x
	Assign_11Assignq/biasPlaceholder_11*
use_locking(*
T0*
_class
loc:@q/bias*
validate_shape(
p
IsVariableInitializedIsVariableInitializedh_common/kernel*
dtype0*"
_class
loc:@h_common/kernel
n
IsVariableInitialized_1IsVariableInitializedh_common/bias* 
_class
loc:@h_common/bias*
dtype0
p
IsVariableInitialized_2IsVariableInitializedh_actor/kernel*!
_class
loc:@h_actor/kernel*
dtype0
l
IsVariableInitialized_3IsVariableInitializedh_actor/bias*
_class
loc:@h_actor/bias*
dtype0
l
IsVariableInitialized_4IsVariableInitializeda_raw/kernel*
_class
loc:@a_raw/kernel*
dtype0
h
IsVariableInitialized_5IsVariableInitialized
a_raw/bias*
_class
loc:@a_raw/bias*
dtype0
t
IsVariableInitialized_6IsVariableInitializedh_common2/kernel*#
_class
loc:@h_common2/kernel*
dtype0
p
IsVariableInitialized_7IsVariableInitializedh_common2/bias*
dtype0*!
_class
loc:@h_common2/bias
r
IsVariableInitialized_8IsVariableInitializedh_critic/kernel*"
_class
loc:@h_critic/kernel*
dtype0
n
IsVariableInitialized_9IsVariableInitializedh_critic/bias* 
_class
loc:@h_critic/bias*
dtype0
e
IsVariableInitialized_10IsVariableInitializedq/kernel*
_class
loc:@q/kernel*
dtype0
a
IsVariableInitialized_11IsVariableInitializedq/bias*
_class
loc:@q/bias*
dtype0
n
IsVariableInitialized_12IsVariableInitializedbeta1_power*!
_class
loc:@h_common2/bias*
dtype0
n
IsVariableInitialized_13IsVariableInitializedbeta2_power*
dtype0*!
_class
loc:@h_common2/bias
z
IsVariableInitialized_14IsVariableInitializedh_common2/kernel/Adam*#
_class
loc:@h_common2/kernel*
dtype0
|
IsVariableInitialized_15IsVariableInitializedh_common2/kernel/Adam_1*#
_class
loc:@h_common2/kernel*
dtype0
v
IsVariableInitialized_16IsVariableInitializedh_common2/bias/Adam*
dtype0*!
_class
loc:@h_common2/bias
x
IsVariableInitialized_17IsVariableInitializedh_common2/bias/Adam_1*!
_class
loc:@h_common2/bias*
dtype0
x
IsVariableInitialized_18IsVariableInitializedh_critic/kernel/Adam*"
_class
loc:@h_critic/kernel*
dtype0
z
IsVariableInitialized_19IsVariableInitializedh_critic/kernel/Adam_1*"
_class
loc:@h_critic/kernel*
dtype0
t
IsVariableInitialized_20IsVariableInitializedh_critic/bias/Adam* 
_class
loc:@h_critic/bias*
dtype0
v
IsVariableInitialized_21IsVariableInitializedh_critic/bias/Adam_1* 
_class
loc:@h_critic/bias*
dtype0
j
IsVariableInitialized_22IsVariableInitializedq/kernel/Adam*
_class
loc:@q/kernel*
dtype0
l
IsVariableInitialized_23IsVariableInitializedq/kernel/Adam_1*
_class
loc:@q/kernel*
dtype0
f
IsVariableInitialized_24IsVariableInitializedq/bias/Adam*
dtype0*
_class
loc:@q/bias
h
IsVariableInitialized_25IsVariableInitializedq/bias/Adam_1*
_class
loc:@q/bias*
dtype0
l
IsVariableInitialized_26IsVariableInitializedbeta1_power_1*
_class
loc:@a_raw/bias*
dtype0
l
IsVariableInitialized_27IsVariableInitializedbeta2_power_1*
_class
loc:@a_raw/bias*
dtype0
x
IsVariableInitialized_28IsVariableInitializedh_common/kernel/Adam*"
_class
loc:@h_common/kernel*
dtype0
z
IsVariableInitialized_29IsVariableInitializedh_common/kernel/Adam_1*"
_class
loc:@h_common/kernel*
dtype0
t
IsVariableInitialized_30IsVariableInitializedh_common/bias/Adam* 
_class
loc:@h_common/bias*
dtype0
v
IsVariableInitialized_31IsVariableInitializedh_common/bias/Adam_1* 
_class
loc:@h_common/bias*
dtype0
v
IsVariableInitialized_32IsVariableInitializedh_actor/kernel/Adam*!
_class
loc:@h_actor/kernel*
dtype0
x
IsVariableInitialized_33IsVariableInitializedh_actor/kernel/Adam_1*
dtype0*!
_class
loc:@h_actor/kernel
r
IsVariableInitialized_34IsVariableInitializedh_actor/bias/Adam*
_class
loc:@h_actor/bias*
dtype0
t
IsVariableInitialized_35IsVariableInitializedh_actor/bias/Adam_1*
dtype0*
_class
loc:@h_actor/bias
r
IsVariableInitialized_36IsVariableInitializeda_raw/kernel/Adam*
_class
loc:@a_raw/kernel*
dtype0
t
IsVariableInitialized_37IsVariableInitializeda_raw/kernel/Adam_1*
_class
loc:@a_raw/kernel*
dtype0
n
IsVariableInitialized_38IsVariableInitializeda_raw/bias/Adam*
_class
loc:@a_raw/bias*
dtype0
p
IsVariableInitialized_39IsVariableInitializeda_raw/bias/Adam_1*
_class
loc:@a_raw/bias*
dtype0

initNoOp^a_raw/bias/Adam/Assign^a_raw/bias/Adam_1/Assign^a_raw/bias/Assign^a_raw/kernel/Adam/Assign^a_raw/kernel/Adam_1/Assign^a_raw/kernel/Assign^beta1_power/Assign^beta1_power_1/Assign^beta2_power/Assign^beta2_power_1/Assign^h_actor/bias/Adam/Assign^h_actor/bias/Adam_1/Assign^h_actor/bias/Assign^h_actor/kernel/Adam/Assign^h_actor/kernel/Adam_1/Assign^h_actor/kernel/Assign^h_common/bias/Adam/Assign^h_common/bias/Adam_1/Assign^h_common/bias/Assign^h_common/kernel/Adam/Assign^h_common/kernel/Adam_1/Assign^h_common/kernel/Assign^h_common2/bias/Adam/Assign^h_common2/bias/Adam_1/Assign^h_common2/bias/Assign^h_common2/kernel/Adam/Assign^h_common2/kernel/Adam_1/Assign^h_common2/kernel/Assign^h_critic/bias/Adam/Assign^h_critic/bias/Adam_1/Assign^h_critic/bias/Assign^h_critic/kernel/Adam/Assign^h_critic/kernel/Adam_1/Assign^h_critic/kernel/Assign^q/bias/Adam/Assign^q/bias/Adam_1/Assign^q/bias/Assign^q/kernel/Adam/Assign^q/kernel/Adam_1/Assign^q/kernel/Assign"