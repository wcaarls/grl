
0
input_1Placeholder*
dtype0*
shape: 
J
s/strided_slice/stackConst*
dtype0*
valueB"        
L
s/strided_slice/stack_1Const*
dtype0*
valueB"       
L
s/strided_slice/stack_2Const*
dtype0*
valueB"      
ë
s/strided_sliceStridedSliceinput_1s/strided_slice/stacks/strided_slice/stack_1s/strided_slice/stack_2*
new_axis_mask *
Index0*

begin_mask*
ellipsis_mask *
end_mask*
T0*
shrink_axis_mask 
J
a/strided_slice/stackConst*
dtype0*
valueB"       
L
a/strided_slice/stack_1Const*
dtype0*
valueB"       
L
a/strided_slice/stack_2Const*
dtype0*
valueB"      
ë
a/strided_sliceStridedSliceinput_1a/strided_slice/stacka/strided_slice/stack_1a/strided_slice/stack_2*
new_axis_mask *
Index0*

begin_mask*
ellipsis_mask *
end_mask*
T0*
shrink_axis_mask 
L
batch_normalization_1/ConstConst*
dtype0*
valueB*  ?
i
batch_normalization_1/gammaVariable*
dtype0*
shape:*
	container *
shared_name 
È
"batch_normalization_1/gamma/AssignAssignbatch_normalization_1/gammabatch_normalization_1/Const*
validate_shape(*.
_class$
" loc:@batch_normalization_1/gamma*
use_locking(*
T0

 batch_normalization_1/gamma/readIdentitybatch_normalization_1/gamma*.
_class$
" loc:@batch_normalization_1/gamma*
T0
N
batch_normalization_1/Const_1Const*
dtype0*
valueB*    
h
batch_normalization_1/betaVariable*
dtype0*
shape:*
	container *
shared_name 
Ç
!batch_normalization_1/beta/AssignAssignbatch_normalization_1/betabatch_normalization_1/Const_1*
validate_shape(*-
_class#
!loc:@batch_normalization_1/beta*
use_locking(*
T0

batch_normalization_1/beta/readIdentitybatch_normalization_1/beta*-
_class#
!loc:@batch_normalization_1/beta*
T0
N
batch_normalization_1/Const_2Const*
dtype0*
valueB*    
o
!batch_normalization_1/moving_meanVariable*
dtype0*
shape:*
	container *
shared_name 
Ü
(batch_normalization_1/moving_mean/AssignAssign!batch_normalization_1/moving_meanbatch_normalization_1/Const_2*
validate_shape(*4
_class*
(&loc:@batch_normalization_1/moving_mean*
use_locking(*
T0

&batch_normalization_1/moving_mean/readIdentity!batch_normalization_1/moving_mean*4
_class*
(&loc:@batch_normalization_1/moving_mean*
T0
N
batch_normalization_1/Const_3Const*
dtype0*
valueB*  ?
s
%batch_normalization_1/moving_varianceVariable*
dtype0*
shape:*
	container *
shared_name 
è
,batch_normalization_1/moving_variance/AssignAssign%batch_normalization_1/moving_variancebatch_normalization_1/Const_3*
validate_shape(*8
_class.
,*loc:@batch_normalization_1/moving_variance*
use_locking(*
T0
 
*batch_normalization_1/moving_variance/readIdentity%batch_normalization_1/moving_variance*8
_class.
,*loc:@batch_normalization_1/moving_variance*
T0
l
9batch_normalization_1/moments/sufficient_statistics/ShapeShapes/strided_slice*
out_type0*
T0
p
Bbatch_normalization_1/moments/sufficient_statistics/Gather/indicesConst*
dtype0*
valueB: 
ú
:batch_normalization_1/moments/sufficient_statistics/GatherGather9batch_normalization_1/moments/sufficient_statistics/ShapeBbatch_normalization_1/moments/sufficient_statistics/Gather/indices*
validate_indices(*
Tparams0*
Tindices0
g
9batch_normalization_1/moments/sufficient_statistics/ConstConst*
dtype0*
valueB: 
Ý
8batch_normalization_1/moments/sufficient_statistics/ProdProd:batch_normalization_1/moments/sufficient_statistics/Gather9batch_normalization_1/moments/sufficient_statistics/Const*
T0*
	keep_dims( *

Tidx0

9batch_normalization_1/moments/sufficient_statistics/countCast8batch_normalization_1/moments/sufficient_statistics/Prod*

DstT0*

SrcT0
^
:batch_normalization_1/moments/sufficient_statistics/SquareSquares/strided_slice*
T0
{
Mbatch_normalization_1/moments/sufficient_statistics/mean_ss/reduction_indicesConst*
dtype0*
valueB: 
È
;batch_normalization_1/moments/sufficient_statistics/mean_ssSums/strided_sliceMbatch_normalization_1/moments/sufficient_statistics/mean_ss/reduction_indices*
T0*
	keep_dims( *

Tidx0
z
Lbatch_normalization_1/moments/sufficient_statistics/var_ss/reduction_indicesConst*
dtype0*
valueB: 
ñ
:batch_normalization_1/moments/sufficient_statistics/var_ssSum:batch_normalization_1/moments/sufficient_statistics/SquareLbatch_normalization_1/moments/sufficient_statistics/var_ss/reduction_indices*
T0*
	keep_dims( *

Tidx0
ü
/batch_normalization_1/moments/normalize/divisor
Reciprocal9batch_normalization_1/moments/sufficient_statistics/count<^batch_normalization_1/moments/sufficient_statistics/mean_ss;^batch_normalization_1/moments/sufficient_statistics/var_ss*
T0
ª
,batch_normalization_1/moments/normalize/meanMul;batch_normalization_1/moments/sufficient_statistics/mean_ss/batch_normalization_1/moments/normalize/divisor*
T0
¨
+batch_normalization_1/moments/normalize/MulMul:batch_normalization_1/moments/sufficient_statistics/var_ss/batch_normalization_1/moments/normalize/divisor*
T0
o
.batch_normalization_1/moments/normalize/SquareSquare,batch_normalization_1/moments/normalize/mean*
T0

0batch_normalization_1/moments/normalize/varianceSub+batch_normalization_1/moments/normalize/Mul.batch_normalization_1/moments/normalize/Square*
T0
R
%batch_normalization_1/batchnorm/add/yConst*
dtype0*
valueB
 *o:

#batch_normalization_1/batchnorm/addAdd0batch_normalization_1/moments/normalize/variance%batch_normalization_1/batchnorm/add/y*
T0
\
%batch_normalization_1/batchnorm/RsqrtRsqrt#batch_normalization_1/batchnorm/add*
T0
|
#batch_normalization_1/batchnorm/mulMul%batch_normalization_1/batchnorm/Rsqrt batch_normalization_1/gamma/read*
T0
k
%batch_normalization_1/batchnorm/mul_1Muls/strided_slice#batch_normalization_1/batchnorm/mul*
T0

%batch_normalization_1/batchnorm/mul_2Mul,batch_normalization_1/moments/normalize/mean#batch_normalization_1/batchnorm/mul*
T0
{
#batch_normalization_1/batchnorm/subSubbatch_normalization_1/beta/read%batch_normalization_1/batchnorm/mul_2*
T0

%batch_normalization_1/batchnorm/add_1Add%batch_normalization_1/batchnorm/mul_1#batch_normalization_1/batchnorm/sub*
T0

+batch_normalization_1/AssignMovingAvg/decayConst*
dtype0*4
_class*
(&loc:@batch_normalization_1/moving_mean*
valueB
 *
×#<
Å
)batch_normalization_1/AssignMovingAvg/subSub&batch_normalization_1/moving_mean/read,batch_normalization_1/moments/normalize/mean*4
_class*
(&loc:@batch_normalization_1/moving_mean*
T0
Ç
)batch_normalization_1/AssignMovingAvg/mulMul)batch_normalization_1/AssignMovingAvg/sub+batch_normalization_1/AssignMovingAvg/decay*4
_class*
(&loc:@batch_normalization_1/moving_mean*
T0
Ò
%batch_normalization_1/AssignMovingAvg	AssignSub!batch_normalization_1/moving_mean)batch_normalization_1/AssignMovingAvg/mul*4
_class*
(&loc:@batch_normalization_1/moving_mean*
use_locking( *
T0

-batch_normalization_1/AssignMovingAvg_1/decayConst*
dtype0*8
_class.
,*loc:@batch_normalization_1/moving_variance*
valueB
 *
×#<
Ó
+batch_normalization_1/AssignMovingAvg_1/subSub*batch_normalization_1/moving_variance/read0batch_normalization_1/moments/normalize/variance*8
_class.
,*loc:@batch_normalization_1/moving_variance*
T0
Ñ
+batch_normalization_1/AssignMovingAvg_1/mulMul+batch_normalization_1/AssignMovingAvg_1/sub-batch_normalization_1/AssignMovingAvg_1/decay*8
_class.
,*loc:@batch_normalization_1/moving_variance*
T0
Þ
'batch_normalization_1/AssignMovingAvg_1	AssignSub%batch_normalization_1/moving_variance+batch_normalization_1/AssignMovingAvg_1/mul*8
_class.
,*loc:@batch_normalization_1/moving_variance*
use_locking( *
T0
S
*batch_normalization_1/keras_learning_phasePlaceholder*
dtype0
*
shape: 

!batch_normalization_1/cond/SwitchSwitch*batch_normalization_1/keras_learning_phase*batch_normalization_1/keras_learning_phase*
T0

]
#batch_normalization_1/cond/switch_tIdentity#batch_normalization_1/cond/Switch:1*
T0

[
#batch_normalization_1/cond/switch_fIdentity!batch_normalization_1/cond/Switch*
T0

c
"batch_normalization_1/cond/pred_idIdentity*batch_normalization_1/keras_learning_phase*
T0

»
#batch_normalization_1/cond/Switch_1Switch%batch_normalization_1/batchnorm/add_1"batch_normalization_1/cond/pred_id*8
_class.
,*loc:@batch_normalization_1/batchnorm/add_1*
T0
}
*batch_normalization_1/cond/batchnorm/add/yConst$^batch_normalization_1/cond/switch_f*
dtype0*
valueB
 *o:
Ì
/batch_normalization_1/cond/batchnorm/add/SwitchSwitch*batch_normalization_1/moving_variance/read"batch_normalization_1/cond/pred_id*8
_class.
,*loc:@batch_normalization_1/moving_variance*
T0

(batch_normalization_1/cond/batchnorm/addAdd/batch_normalization_1/cond/batchnorm/add/Switch*batch_normalization_1/cond/batchnorm/add/y*
T0
f
*batch_normalization_1/cond/batchnorm/RsqrtRsqrt(batch_normalization_1/cond/batchnorm/add*
T0
¸
/batch_normalization_1/cond/batchnorm/mul/SwitchSwitch batch_normalization_1/gamma/read"batch_normalization_1/cond/pred_id*.
_class$
" loc:@batch_normalization_1/gamma*
T0

(batch_normalization_1/cond/batchnorm/mulMul*batch_normalization_1/cond/batchnorm/Rsqrt/batch_normalization_1/cond/batchnorm/mul/Switch*
T0

1batch_normalization_1/cond/batchnorm/mul_1/SwitchSwitchs/strided_slice"batch_normalization_1/cond/pred_id*"
_class
loc:@s/strided_slice*
T0

*batch_normalization_1/cond/batchnorm/mul_1Mul1batch_normalization_1/cond/batchnorm/mul_1/Switch(batch_normalization_1/cond/batchnorm/mul*
T0
Æ
1batch_normalization_1/cond/batchnorm/mul_2/SwitchSwitch&batch_normalization_1/moving_mean/read"batch_normalization_1/cond/pred_id*4
_class*
(&loc:@batch_normalization_1/moving_mean*
T0

*batch_normalization_1/cond/batchnorm/mul_2Mul1batch_normalization_1/cond/batchnorm/mul_2/Switch(batch_normalization_1/cond/batchnorm/mul*
T0
¶
/batch_normalization_1/cond/batchnorm/sub/SwitchSwitchbatch_normalization_1/beta/read"batch_normalization_1/cond/pred_id*-
_class#
!loc:@batch_normalization_1/beta*
T0

(batch_normalization_1/cond/batchnorm/subSub/batch_normalization_1/cond/batchnorm/sub/Switch*batch_normalization_1/cond/batchnorm/mul_2*
T0

*batch_normalization_1/cond/batchnorm/add_1Add*batch_normalization_1/cond/batchnorm/mul_1(batch_normalization_1/cond/batchnorm/sub*
T0

 batch_normalization_1/cond/MergeMerge*batch_normalization_1/cond/batchnorm/add_1%batch_normalization_1/cond/Switch_1:1*
T0*
N
L
h1/random_uniform/shapeConst*
dtype0*
valueB"   d   
B
h1/random_uniform/minConst*
dtype0*
valueB
 *B[x¾
B
h1/random_uniform/maxConst*
dtype0*
valueB
 *B[x>
~
h1/random_uniform/RandomUniformRandomUniformh1/random_uniform/shape*
dtype0*
seed2Ê«*
seed±ÿå)*
T0
S
h1/random_uniform/subSubh1/random_uniform/maxh1/random_uniform/min*
T0
]
h1/random_uniform/mulMulh1/random_uniform/RandomUniformh1/random_uniform/sub*
T0
O
h1/random_uniformAddh1/random_uniform/mulh1/random_uniform/min*
T0
[
	h1/kernelVariable*
dtype0*
shape
:d*
	container *
shared_name 

h1/kernel/AssignAssign	h1/kernelh1/random_uniform*
validate_shape(*
_class
loc:@h1/kernel*
use_locking(*
T0
L
h1/kernel/readIdentity	h1/kernel*
_class
loc:@h1/kernel*
T0
9
h1/ConstConst*
dtype0*
valueBd*    
U
h1/biasVariable*
dtype0*
shape:d*
	container *
shared_name 
y
h1/bias/AssignAssignh1/biash1/Const*
validate_shape(*
_class
loc:@h1/bias*
use_locking(*
T0
F
h1/bias/readIdentityh1/bias*
_class
loc:@h1/bias*
T0
t
	h1/MatMulMatMul batch_normalization_1/cond/Mergeh1/kernel/read*
transpose_b( *
transpose_a( *
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
batch_normalization_2/ConstConst*
dtype0*
valueBd*  ?
i
batch_normalization_2/gammaVariable*
dtype0*
shape:d*
	container *
shared_name 
È
"batch_normalization_2/gamma/AssignAssignbatch_normalization_2/gammabatch_normalization_2/Const*
validate_shape(*.
_class$
" loc:@batch_normalization_2/gamma*
use_locking(*
T0

 batch_normalization_2/gamma/readIdentitybatch_normalization_2/gamma*.
_class$
" loc:@batch_normalization_2/gamma*
T0
N
batch_normalization_2/Const_1Const*
dtype0*
valueBd*    
h
batch_normalization_2/betaVariable*
dtype0*
shape:d*
	container *
shared_name 
Ç
!batch_normalization_2/beta/AssignAssignbatch_normalization_2/betabatch_normalization_2/Const_1*
validate_shape(*-
_class#
!loc:@batch_normalization_2/beta*
use_locking(*
T0

batch_normalization_2/beta/readIdentitybatch_normalization_2/beta*-
_class#
!loc:@batch_normalization_2/beta*
T0
N
batch_normalization_2/Const_2Const*
dtype0*
valueBd*    
o
!batch_normalization_2/moving_meanVariable*
dtype0*
shape:d*
	container *
shared_name 
Ü
(batch_normalization_2/moving_mean/AssignAssign!batch_normalization_2/moving_meanbatch_normalization_2/Const_2*
validate_shape(*4
_class*
(&loc:@batch_normalization_2/moving_mean*
use_locking(*
T0

&batch_normalization_2/moving_mean/readIdentity!batch_normalization_2/moving_mean*4
_class*
(&loc:@batch_normalization_2/moving_mean*
T0
N
batch_normalization_2/Const_3Const*
dtype0*
valueBd*  ?
s
%batch_normalization_2/moving_varianceVariable*
dtype0*
shape:d*
	container *
shared_name 
è
,batch_normalization_2/moving_variance/AssignAssign%batch_normalization_2/moving_variancebatch_normalization_2/Const_3*
validate_shape(*8
_class.
,*loc:@batch_normalization_2/moving_variance*
use_locking(*
T0
 
*batch_normalization_2/moving_variance/readIdentity%batch_normalization_2/moving_variance*8
_class.
,*loc:@batch_normalization_2/moving_variance*
T0
d
9batch_normalization_2/moments/sufficient_statistics/ShapeShapeh1/Relu*
out_type0*
T0
p
Bbatch_normalization_2/moments/sufficient_statistics/Gather/indicesConst*
dtype0*
valueB: 
ú
:batch_normalization_2/moments/sufficient_statistics/GatherGather9batch_normalization_2/moments/sufficient_statistics/ShapeBbatch_normalization_2/moments/sufficient_statistics/Gather/indices*
validate_indices(*
Tparams0*
Tindices0
g
9batch_normalization_2/moments/sufficient_statistics/ConstConst*
dtype0*
valueB: 
Ý
8batch_normalization_2/moments/sufficient_statistics/ProdProd:batch_normalization_2/moments/sufficient_statistics/Gather9batch_normalization_2/moments/sufficient_statistics/Const*
T0*
	keep_dims( *

Tidx0

9batch_normalization_2/moments/sufficient_statistics/countCast8batch_normalization_2/moments/sufficient_statistics/Prod*

DstT0*

SrcT0
V
:batch_normalization_2/moments/sufficient_statistics/SquareSquareh1/Relu*
T0
{
Mbatch_normalization_2/moments/sufficient_statistics/mean_ss/reduction_indicesConst*
dtype0*
valueB: 
À
;batch_normalization_2/moments/sufficient_statistics/mean_ssSumh1/ReluMbatch_normalization_2/moments/sufficient_statistics/mean_ss/reduction_indices*
T0*
	keep_dims( *

Tidx0
z
Lbatch_normalization_2/moments/sufficient_statistics/var_ss/reduction_indicesConst*
dtype0*
valueB: 
ñ
:batch_normalization_2/moments/sufficient_statistics/var_ssSum:batch_normalization_2/moments/sufficient_statistics/SquareLbatch_normalization_2/moments/sufficient_statistics/var_ss/reduction_indices*
T0*
	keep_dims( *

Tidx0
ü
/batch_normalization_2/moments/normalize/divisor
Reciprocal9batch_normalization_2/moments/sufficient_statistics/count<^batch_normalization_2/moments/sufficient_statistics/mean_ss;^batch_normalization_2/moments/sufficient_statistics/var_ss*
T0
ª
,batch_normalization_2/moments/normalize/meanMul;batch_normalization_2/moments/sufficient_statistics/mean_ss/batch_normalization_2/moments/normalize/divisor*
T0
¨
+batch_normalization_2/moments/normalize/MulMul:batch_normalization_2/moments/sufficient_statistics/var_ss/batch_normalization_2/moments/normalize/divisor*
T0
o
.batch_normalization_2/moments/normalize/SquareSquare,batch_normalization_2/moments/normalize/mean*
T0

0batch_normalization_2/moments/normalize/varianceSub+batch_normalization_2/moments/normalize/Mul.batch_normalization_2/moments/normalize/Square*
T0
R
%batch_normalization_2/batchnorm/add/yConst*
dtype0*
valueB
 *o:

#batch_normalization_2/batchnorm/addAdd0batch_normalization_2/moments/normalize/variance%batch_normalization_2/batchnorm/add/y*
T0
\
%batch_normalization_2/batchnorm/RsqrtRsqrt#batch_normalization_2/batchnorm/add*
T0
|
#batch_normalization_2/batchnorm/mulMul%batch_normalization_2/batchnorm/Rsqrt batch_normalization_2/gamma/read*
T0
c
%batch_normalization_2/batchnorm/mul_1Mulh1/Relu#batch_normalization_2/batchnorm/mul*
T0

%batch_normalization_2/batchnorm/mul_2Mul,batch_normalization_2/moments/normalize/mean#batch_normalization_2/batchnorm/mul*
T0
{
#batch_normalization_2/batchnorm/subSubbatch_normalization_2/beta/read%batch_normalization_2/batchnorm/mul_2*
T0

%batch_normalization_2/batchnorm/add_1Add%batch_normalization_2/batchnorm/mul_1#batch_normalization_2/batchnorm/sub*
T0

+batch_normalization_2/AssignMovingAvg/decayConst*
dtype0*4
_class*
(&loc:@batch_normalization_2/moving_mean*
valueB
 *
×#<
Å
)batch_normalization_2/AssignMovingAvg/subSub&batch_normalization_2/moving_mean/read,batch_normalization_2/moments/normalize/mean*4
_class*
(&loc:@batch_normalization_2/moving_mean*
T0
Ç
)batch_normalization_2/AssignMovingAvg/mulMul)batch_normalization_2/AssignMovingAvg/sub+batch_normalization_2/AssignMovingAvg/decay*4
_class*
(&loc:@batch_normalization_2/moving_mean*
T0
Ò
%batch_normalization_2/AssignMovingAvg	AssignSub!batch_normalization_2/moving_mean)batch_normalization_2/AssignMovingAvg/mul*4
_class*
(&loc:@batch_normalization_2/moving_mean*
use_locking( *
T0

-batch_normalization_2/AssignMovingAvg_1/decayConst*
dtype0*8
_class.
,*loc:@batch_normalization_2/moving_variance*
valueB
 *
×#<
Ó
+batch_normalization_2/AssignMovingAvg_1/subSub*batch_normalization_2/moving_variance/read0batch_normalization_2/moments/normalize/variance*8
_class.
,*loc:@batch_normalization_2/moving_variance*
T0
Ñ
+batch_normalization_2/AssignMovingAvg_1/mulMul+batch_normalization_2/AssignMovingAvg_1/sub-batch_normalization_2/AssignMovingAvg_1/decay*8
_class.
,*loc:@batch_normalization_2/moving_variance*
T0
Þ
'batch_normalization_2/AssignMovingAvg_1	AssignSub%batch_normalization_2/moving_variance+batch_normalization_2/AssignMovingAvg_1/mul*8
_class.
,*loc:@batch_normalization_2/moving_variance*
use_locking( *
T0

!batch_normalization_2/cond/SwitchSwitch*batch_normalization_1/keras_learning_phase*batch_normalization_1/keras_learning_phase*
T0

]
#batch_normalization_2/cond/switch_tIdentity#batch_normalization_2/cond/Switch:1*
T0

[
#batch_normalization_2/cond/switch_fIdentity!batch_normalization_2/cond/Switch*
T0

c
"batch_normalization_2/cond/pred_idIdentity*batch_normalization_1/keras_learning_phase*
T0

»
#batch_normalization_2/cond/Switch_1Switch%batch_normalization_2/batchnorm/add_1"batch_normalization_2/cond/pred_id*8
_class.
,*loc:@batch_normalization_2/batchnorm/add_1*
T0
}
*batch_normalization_2/cond/batchnorm/add/yConst$^batch_normalization_2/cond/switch_f*
dtype0*
valueB
 *o:
Ì
/batch_normalization_2/cond/batchnorm/add/SwitchSwitch*batch_normalization_2/moving_variance/read"batch_normalization_2/cond/pred_id*8
_class.
,*loc:@batch_normalization_2/moving_variance*
T0

(batch_normalization_2/cond/batchnorm/addAdd/batch_normalization_2/cond/batchnorm/add/Switch*batch_normalization_2/cond/batchnorm/add/y*
T0
f
*batch_normalization_2/cond/batchnorm/RsqrtRsqrt(batch_normalization_2/cond/batchnorm/add*
T0
¸
/batch_normalization_2/cond/batchnorm/mul/SwitchSwitch batch_normalization_2/gamma/read"batch_normalization_2/cond/pred_id*.
_class$
" loc:@batch_normalization_2/gamma*
T0

(batch_normalization_2/cond/batchnorm/mulMul*batch_normalization_2/cond/batchnorm/Rsqrt/batch_normalization_2/cond/batchnorm/mul/Switch*
T0

1batch_normalization_2/cond/batchnorm/mul_1/SwitchSwitchh1/Relu"batch_normalization_2/cond/pred_id*
_class
loc:@h1/Relu*
T0

*batch_normalization_2/cond/batchnorm/mul_1Mul1batch_normalization_2/cond/batchnorm/mul_1/Switch(batch_normalization_2/cond/batchnorm/mul*
T0
Æ
1batch_normalization_2/cond/batchnorm/mul_2/SwitchSwitch&batch_normalization_2/moving_mean/read"batch_normalization_2/cond/pred_id*4
_class*
(&loc:@batch_normalization_2/moving_mean*
T0

*batch_normalization_2/cond/batchnorm/mul_2Mul1batch_normalization_2/cond/batchnorm/mul_2/Switch(batch_normalization_2/cond/batchnorm/mul*
T0
¶
/batch_normalization_2/cond/batchnorm/sub/SwitchSwitchbatch_normalization_2/beta/read"batch_normalization_2/cond/pred_id*-
_class#
!loc:@batch_normalization_2/beta*
T0

(batch_normalization_2/cond/batchnorm/subSub/batch_normalization_2/cond/batchnorm/sub/Switch*batch_normalization_2/cond/batchnorm/mul_2*
T0

*batch_normalization_2/cond/batchnorm/add_1Add*batch_normalization_2/cond/batchnorm/mul_1(batch_normalization_2/cond/batchnorm/sub*
T0

 batch_normalization_2/cond/MergeMerge*batch_normalization_2/cond/batchnorm/add_1%batch_normalization_2/cond/Switch_1:1*
T0*
N
L
h2/random_uniform/shapeConst*
dtype0*
valueB"d   d   
B
h2/random_uniform/minConst*
dtype0*
valueB
 *¬\1¾
B
h2/random_uniform/maxConst*
dtype0*
valueB
 *¬\1>
~
h2/random_uniform/RandomUniformRandomUniformh2/random_uniform/shape*
dtype0*
seed2ÓÑ*
seed±ÿå)*
T0
S
h2/random_uniform/subSubh2/random_uniform/maxh2/random_uniform/min*
T0
]
h2/random_uniform/mulMulh2/random_uniform/RandomUniformh2/random_uniform/sub*
T0
O
h2/random_uniformAddh2/random_uniform/mulh2/random_uniform/min*
T0
[
	h2/kernelVariable*
dtype0*
shape
:dd*
	container *
shared_name 

h2/kernel/AssignAssign	h2/kernelh2/random_uniform*
validate_shape(*
_class
loc:@h2/kernel*
use_locking(*
T0
L
h2/kernel/readIdentity	h2/kernel*
_class
loc:@h2/kernel*
T0
9
h2/ConstConst*
dtype0*
valueBd*    
U
h2/biasVariable*
dtype0*
shape:d*
	container *
shared_name 
y
h2/bias/AssignAssignh2/biash2/Const*
validate_shape(*
_class
loc:@h2/bias*
use_locking(*
T0
F
h2/bias/readIdentityh2/bias*
_class
loc:@h2/bias*
T0
t
	h2/MatMulMatMul batch_normalization_2/cond/Mergeh2/kernel/read*
transpose_b( *
transpose_a( *
T0
N

h2/BiasAddBiasAdd	h2/MatMulh2/bias/read*
T0*
data_formatNHWC
$
h2/ReluRelu
h2/BiasAdd*
T0
L
batch_normalization_3/ConstConst*
dtype0*
valueBd*  ?
i
batch_normalization_3/gammaVariable*
dtype0*
shape:d*
	container *
shared_name 
È
"batch_normalization_3/gamma/AssignAssignbatch_normalization_3/gammabatch_normalization_3/Const*
validate_shape(*.
_class$
" loc:@batch_normalization_3/gamma*
use_locking(*
T0

 batch_normalization_3/gamma/readIdentitybatch_normalization_3/gamma*.
_class$
" loc:@batch_normalization_3/gamma*
T0
N
batch_normalization_3/Const_1Const*
dtype0*
valueBd*    
h
batch_normalization_3/betaVariable*
dtype0*
shape:d*
	container *
shared_name 
Ç
!batch_normalization_3/beta/AssignAssignbatch_normalization_3/betabatch_normalization_3/Const_1*
validate_shape(*-
_class#
!loc:@batch_normalization_3/beta*
use_locking(*
T0

batch_normalization_3/beta/readIdentitybatch_normalization_3/beta*-
_class#
!loc:@batch_normalization_3/beta*
T0
N
batch_normalization_3/Const_2Const*
dtype0*
valueBd*    
o
!batch_normalization_3/moving_meanVariable*
dtype0*
shape:d*
	container *
shared_name 
Ü
(batch_normalization_3/moving_mean/AssignAssign!batch_normalization_3/moving_meanbatch_normalization_3/Const_2*
validate_shape(*4
_class*
(&loc:@batch_normalization_3/moving_mean*
use_locking(*
T0

&batch_normalization_3/moving_mean/readIdentity!batch_normalization_3/moving_mean*4
_class*
(&loc:@batch_normalization_3/moving_mean*
T0
N
batch_normalization_3/Const_3Const*
dtype0*
valueBd*  ?
s
%batch_normalization_3/moving_varianceVariable*
dtype0*
shape:d*
	container *
shared_name 
è
,batch_normalization_3/moving_variance/AssignAssign%batch_normalization_3/moving_variancebatch_normalization_3/Const_3*
validate_shape(*8
_class.
,*loc:@batch_normalization_3/moving_variance*
use_locking(*
T0
 
*batch_normalization_3/moving_variance/readIdentity%batch_normalization_3/moving_variance*8
_class.
,*loc:@batch_normalization_3/moving_variance*
T0
d
9batch_normalization_3/moments/sufficient_statistics/ShapeShapeh2/Relu*
out_type0*
T0
p
Bbatch_normalization_3/moments/sufficient_statistics/Gather/indicesConst*
dtype0*
valueB: 
ú
:batch_normalization_3/moments/sufficient_statistics/GatherGather9batch_normalization_3/moments/sufficient_statistics/ShapeBbatch_normalization_3/moments/sufficient_statistics/Gather/indices*
validate_indices(*
Tparams0*
Tindices0
g
9batch_normalization_3/moments/sufficient_statistics/ConstConst*
dtype0*
valueB: 
Ý
8batch_normalization_3/moments/sufficient_statistics/ProdProd:batch_normalization_3/moments/sufficient_statistics/Gather9batch_normalization_3/moments/sufficient_statistics/Const*
T0*
	keep_dims( *

Tidx0

9batch_normalization_3/moments/sufficient_statistics/countCast8batch_normalization_3/moments/sufficient_statistics/Prod*

DstT0*

SrcT0
V
:batch_normalization_3/moments/sufficient_statistics/SquareSquareh2/Relu*
T0
{
Mbatch_normalization_3/moments/sufficient_statistics/mean_ss/reduction_indicesConst*
dtype0*
valueB: 
À
;batch_normalization_3/moments/sufficient_statistics/mean_ssSumh2/ReluMbatch_normalization_3/moments/sufficient_statistics/mean_ss/reduction_indices*
T0*
	keep_dims( *

Tidx0
z
Lbatch_normalization_3/moments/sufficient_statistics/var_ss/reduction_indicesConst*
dtype0*
valueB: 
ñ
:batch_normalization_3/moments/sufficient_statistics/var_ssSum:batch_normalization_3/moments/sufficient_statistics/SquareLbatch_normalization_3/moments/sufficient_statistics/var_ss/reduction_indices*
T0*
	keep_dims( *

Tidx0
ü
/batch_normalization_3/moments/normalize/divisor
Reciprocal9batch_normalization_3/moments/sufficient_statistics/count<^batch_normalization_3/moments/sufficient_statistics/mean_ss;^batch_normalization_3/moments/sufficient_statistics/var_ss*
T0
ª
,batch_normalization_3/moments/normalize/meanMul;batch_normalization_3/moments/sufficient_statistics/mean_ss/batch_normalization_3/moments/normalize/divisor*
T0
¨
+batch_normalization_3/moments/normalize/MulMul:batch_normalization_3/moments/sufficient_statistics/var_ss/batch_normalization_3/moments/normalize/divisor*
T0
o
.batch_normalization_3/moments/normalize/SquareSquare,batch_normalization_3/moments/normalize/mean*
T0

0batch_normalization_3/moments/normalize/varianceSub+batch_normalization_3/moments/normalize/Mul.batch_normalization_3/moments/normalize/Square*
T0
R
%batch_normalization_3/batchnorm/add/yConst*
dtype0*
valueB
 *o:

#batch_normalization_3/batchnorm/addAdd0batch_normalization_3/moments/normalize/variance%batch_normalization_3/batchnorm/add/y*
T0
\
%batch_normalization_3/batchnorm/RsqrtRsqrt#batch_normalization_3/batchnorm/add*
T0
|
#batch_normalization_3/batchnorm/mulMul%batch_normalization_3/batchnorm/Rsqrt batch_normalization_3/gamma/read*
T0
c
%batch_normalization_3/batchnorm/mul_1Mulh2/Relu#batch_normalization_3/batchnorm/mul*
T0

%batch_normalization_3/batchnorm/mul_2Mul,batch_normalization_3/moments/normalize/mean#batch_normalization_3/batchnorm/mul*
T0
{
#batch_normalization_3/batchnorm/subSubbatch_normalization_3/beta/read%batch_normalization_3/batchnorm/mul_2*
T0

%batch_normalization_3/batchnorm/add_1Add%batch_normalization_3/batchnorm/mul_1#batch_normalization_3/batchnorm/sub*
T0

+batch_normalization_3/AssignMovingAvg/decayConst*
dtype0*4
_class*
(&loc:@batch_normalization_3/moving_mean*
valueB
 *
×#<
Å
)batch_normalization_3/AssignMovingAvg/subSub&batch_normalization_3/moving_mean/read,batch_normalization_3/moments/normalize/mean*4
_class*
(&loc:@batch_normalization_3/moving_mean*
T0
Ç
)batch_normalization_3/AssignMovingAvg/mulMul)batch_normalization_3/AssignMovingAvg/sub+batch_normalization_3/AssignMovingAvg/decay*4
_class*
(&loc:@batch_normalization_3/moving_mean*
T0
Ò
%batch_normalization_3/AssignMovingAvg	AssignSub!batch_normalization_3/moving_mean)batch_normalization_3/AssignMovingAvg/mul*4
_class*
(&loc:@batch_normalization_3/moving_mean*
use_locking( *
T0

-batch_normalization_3/AssignMovingAvg_1/decayConst*
dtype0*8
_class.
,*loc:@batch_normalization_3/moving_variance*
valueB
 *
×#<
Ó
+batch_normalization_3/AssignMovingAvg_1/subSub*batch_normalization_3/moving_variance/read0batch_normalization_3/moments/normalize/variance*8
_class.
,*loc:@batch_normalization_3/moving_variance*
T0
Ñ
+batch_normalization_3/AssignMovingAvg_1/mulMul+batch_normalization_3/AssignMovingAvg_1/sub-batch_normalization_3/AssignMovingAvg_1/decay*8
_class.
,*loc:@batch_normalization_3/moving_variance*
T0
Þ
'batch_normalization_3/AssignMovingAvg_1	AssignSub%batch_normalization_3/moving_variance+batch_normalization_3/AssignMovingAvg_1/mul*8
_class.
,*loc:@batch_normalization_3/moving_variance*
use_locking( *
T0

!batch_normalization_3/cond/SwitchSwitch*batch_normalization_1/keras_learning_phase*batch_normalization_1/keras_learning_phase*
T0

]
#batch_normalization_3/cond/switch_tIdentity#batch_normalization_3/cond/Switch:1*
T0

[
#batch_normalization_3/cond/switch_fIdentity!batch_normalization_3/cond/Switch*
T0

c
"batch_normalization_3/cond/pred_idIdentity*batch_normalization_1/keras_learning_phase*
T0

»
#batch_normalization_3/cond/Switch_1Switch%batch_normalization_3/batchnorm/add_1"batch_normalization_3/cond/pred_id*8
_class.
,*loc:@batch_normalization_3/batchnorm/add_1*
T0
}
*batch_normalization_3/cond/batchnorm/add/yConst$^batch_normalization_3/cond/switch_f*
dtype0*
valueB
 *o:
Ì
/batch_normalization_3/cond/batchnorm/add/SwitchSwitch*batch_normalization_3/moving_variance/read"batch_normalization_3/cond/pred_id*8
_class.
,*loc:@batch_normalization_3/moving_variance*
T0

(batch_normalization_3/cond/batchnorm/addAdd/batch_normalization_3/cond/batchnorm/add/Switch*batch_normalization_3/cond/batchnorm/add/y*
T0
f
*batch_normalization_3/cond/batchnorm/RsqrtRsqrt(batch_normalization_3/cond/batchnorm/add*
T0
¸
/batch_normalization_3/cond/batchnorm/mul/SwitchSwitch batch_normalization_3/gamma/read"batch_normalization_3/cond/pred_id*.
_class$
" loc:@batch_normalization_3/gamma*
T0

(batch_normalization_3/cond/batchnorm/mulMul*batch_normalization_3/cond/batchnorm/Rsqrt/batch_normalization_3/cond/batchnorm/mul/Switch*
T0

1batch_normalization_3/cond/batchnorm/mul_1/SwitchSwitchh2/Relu"batch_normalization_3/cond/pred_id*
_class
loc:@h2/Relu*
T0

*batch_normalization_3/cond/batchnorm/mul_1Mul1batch_normalization_3/cond/batchnorm/mul_1/Switch(batch_normalization_3/cond/batchnorm/mul*
T0
Æ
1batch_normalization_3/cond/batchnorm/mul_2/SwitchSwitch&batch_normalization_3/moving_mean/read"batch_normalization_3/cond/pred_id*4
_class*
(&loc:@batch_normalization_3/moving_mean*
T0

*batch_normalization_3/cond/batchnorm/mul_2Mul1batch_normalization_3/cond/batchnorm/mul_2/Switch(batch_normalization_3/cond/batchnorm/mul*
T0
¶
/batch_normalization_3/cond/batchnorm/sub/SwitchSwitchbatch_normalization_3/beta/read"batch_normalization_3/cond/pred_id*-
_class#
!loc:@batch_normalization_3/beta*
T0

(batch_normalization_3/cond/batchnorm/subSub/batch_normalization_3/cond/batchnorm/sub/Switch*batch_normalization_3/cond/batchnorm/mul_2*
T0

*batch_normalization_3/cond/batchnorm/add_1Add*batch_normalization_3/cond/batchnorm/mul_1(batch_normalization_3/cond/batchnorm/sub*
T0

 batch_normalization_3/cond/MergeMerge*batch_normalization_3/cond/batchnorm/add_1%batch_normalization_3/cond/Switch_1:1*
T0*
N
K
V/random_uniform/shapeConst*
dtype0*
valueB"d      
A
V/random_uniform/minConst*
dtype0*
valueB
 *<y¾
A
V/random_uniform/maxConst*
dtype0*
valueB
 *<y>
|
V/random_uniform/RandomUniformRandomUniformV/random_uniform/shape*
dtype0*
seed2­­Ë*
seed±ÿå)*
T0
P
V/random_uniform/subSubV/random_uniform/maxV/random_uniform/min*
T0
Z
V/random_uniform/mulMulV/random_uniform/RandomUniformV/random_uniform/sub*
T0
L
V/random_uniformAddV/random_uniform/mulV/random_uniform/min*
T0
Z
V/kernelVariable*
dtype0*
shape
:d*
	container *
shared_name 

V/kernel/AssignAssignV/kernelV/random_uniform*
validate_shape(*
_class
loc:@V/kernel*
use_locking(*
T0
I
V/kernel/readIdentityV/kernel*
_class
loc:@V/kernel*
T0
8
V/ConstConst*
dtype0*
valueB*    
T
V/biasVariable*
dtype0*
shape:*
	container *
shared_name 
u
V/bias/AssignAssignV/biasV/Const*
validate_shape(*
_class
loc:@V/bias*
use_locking(*
T0
C
V/bias/readIdentityV/bias*
_class
loc:@V/bias*
T0
r
V/MatMulMatMul batch_normalization_3/cond/MergeV/kernel/read*
transpose_b( *
transpose_a( *
T0
K
	V/BiasAddBiasAddV/MatMulV/bias/read*
T0*
data_formatNHWC
Q
dense_1/random_uniform/shapeConst*
dtype0*
valueB"d      
G
dense_1/random_uniform/minConst*
dtype0*
valueB
 *<y¾
G
dense_1/random_uniform/maxConst*
dtype0*
valueB
 *<y>

$dense_1/random_uniform/RandomUniformRandomUniformdense_1/random_uniform/shape*
dtype0*
seed2²*
seed±ÿå)*
T0
b
dense_1/random_uniform/subSubdense_1/random_uniform/maxdense_1/random_uniform/min*
T0
l
dense_1/random_uniform/mulMul$dense_1/random_uniform/RandomUniformdense_1/random_uniform/sub*
T0
^
dense_1/random_uniformAdddense_1/random_uniform/muldense_1/random_uniform/min*
T0
`
dense_1/kernelVariable*
dtype0*
shape
:d*
	container *
shared_name 

dense_1/kernel/AssignAssigndense_1/kerneldense_1/random_uniform*
validate_shape(*!
_class
loc:@dense_1/kernel*
use_locking(*
T0
[
dense_1/kernel/readIdentitydense_1/kernel*!
_class
loc:@dense_1/kernel*
T0
>
dense_1/ConstConst*
dtype0*
valueB*    
Z
dense_1/biasVariable*
dtype0*
shape:*
	container *
shared_name 

dense_1/bias/AssignAssigndense_1/biasdense_1/Const*
validate_shape(*
_class
loc:@dense_1/bias*
use_locking(*
T0
U
dense_1/bias/readIdentitydense_1/bias*
_class
loc:@dense_1/bias*
T0
~
dense_1/MatMulMatMul batch_normalization_3/cond/Mergedense_1/kernel/read*
transpose_b( *
transpose_a( *
T0
]
dense_1/BiasAddBiasAdddense_1/MatMuldense_1/bias/read*
T0*
data_formatNHWC
&
P/ExpExpdense_1/BiasAdd*
T0
"
P/SquareSquareP/Exp*
T0
6
P/PlaceholderPlaceholder*
dtype0*
shape: 
&
P/Exp_1ExpP/Placeholder*
T0
&

P/Square_1SquareP/Exp_1*
T0
Q
dense_2/random_uniform/shapeConst*
dtype0*
valueB"d      
G
dense_2/random_uniform/minConst*
dtype0*
valueB
 *<y¾
G
dense_2/random_uniform/maxConst*
dtype0*
valueB
 *<y>

$dense_2/random_uniform/RandomUniformRandomUniformdense_2/random_uniform/shape*
dtype0*
seed2çÕÿ*
seed±ÿå)*
T0
b
dense_2/random_uniform/subSubdense_2/random_uniform/maxdense_2/random_uniform/min*
T0
l
dense_2/random_uniform/mulMul$dense_2/random_uniform/RandomUniformdense_2/random_uniform/sub*
T0
^
dense_2/random_uniformAdddense_2/random_uniform/muldense_2/random_uniform/min*
T0
`
dense_2/kernelVariable*
dtype0*
shape
:d*
	container *
shared_name 

dense_2/kernel/AssignAssigndense_2/kerneldense_2/random_uniform*
validate_shape(*!
_class
loc:@dense_2/kernel*
use_locking(*
T0
[
dense_2/kernel/readIdentitydense_2/kernel*!
_class
loc:@dense_2/kernel*
T0
>
dense_2/ConstConst*
dtype0*
valueB*    
Z
dense_2/biasVariable*
dtype0*
shape:*
	container *
shared_name 

dense_2/bias/AssignAssigndense_2/biasdense_2/Const*
validate_shape(*
_class
loc:@dense_2/bias*
use_locking(*
T0
U
dense_2/bias/readIdentitydense_2/bias*
_class
loc:@dense_2/bias*
T0
~
dense_2/MatMulMatMul batch_normalization_3/cond/Mergedense_2/kernel/read*
transpose_b( *
transpose_a( *
T0
]
dense_2/BiasAddBiasAdddense_2/MatMuldense_2/bias/read*
T0*
data_formatNHWC
.
dense_2/TanhTanhdense_2/BiasAdd*
T0
5
mu/mul/xConst*
dtype0*
valueB
 *  @@
.
mu/mulMulmu/mul/xdense_2/Tanh*
T0
7
mu/PlaceholderPlaceholder*
dtype0*
shape: 
7

mu/mul_1/xConst*
dtype0*
valueB
 *  @@
4
mu/mul_1Mul
mu/mul_1/xmu/Placeholder*
T0
$
lambda_1/NegNegmu/mul*
T0
=
lambda_1/PlaceholderPlaceholder*
dtype0*
shape: 
4
lambda_1/Neg_1Neglambda_1/Placeholder*
T0
8
	add_1/addAdda/strided_slicelambda_1/Neg*
T0
-
lambda_2/SquareSquare	add_1/add*
T0
;
lambda_2/mul/xConst*
dtype0*
valueB
 *   ¿
=
lambda_2/mulMullambda_2/mul/xlambda_2/Square*
T0
=
lambda_2/PlaceholderPlaceholder*
dtype0*
shape: 
:
lambda_2/Square_1Squarelambda_2/Placeholder*
T0
=
lambda_2/mul_1/xConst*
dtype0*
valueB
 *   ¿
C
lambda_2/mul_1Mullambda_2/mul_1/xlambda_2/Square_1*
T0
-
A/mulMulP/Squarelambda_2/mul*
T0
'
Q/addAdd	V/BiasAddA/mul*
T0
E
iterations/initial_valueConst*
dtype0*
valueB
 *    
T

iterationsVariable*
dtype0*
shape: *
	container *
shared_name 

iterations/AssignAssign
iterationsiterations/initial_value*
validate_shape(*
_class
loc:@iterations*
use_locking(*
T0
O
iterations/readIdentity
iterations*
_class
loc:@iterations*
T0
=
lr/initial_valueConst*
dtype0*
valueB
 *o:
L
lrVariable*
dtype0*
shape: *
	container *
shared_name 
r
	lr/AssignAssignlrlr/initial_value*
validate_shape(*
_class
	loc:@lr*
use_locking(*
T0
7
lr/readIdentitylr*
_class
	loc:@lr*
T0
A
beta_1/initial_valueConst*
dtype0*
valueB
 *fff?
P
beta_1Variable*
dtype0*
shape: *
	container *
shared_name 

beta_1/AssignAssignbeta_1beta_1/initial_value*
validate_shape(*
_class
loc:@beta_1*
use_locking(*
T0
C
beta_1/readIdentitybeta_1*
_class
loc:@beta_1*
T0
A
beta_2/initial_valueConst*
dtype0*
valueB
 *w¾?
P
beta_2Variable*
dtype0*
shape: *
	container *
shared_name 

beta_2/AssignAssignbeta_2beta_2/initial_value*
validate_shape(*
_class
loc:@beta_2*
use_locking(*
T0
C
beta_2/readIdentitybeta_2*
_class
loc:@beta_2*
T0
@
decay/initial_valueConst*
dtype0*
valueB
 *    
O
decayVariable*
dtype0*
shape: *
	container *
shared_name 
~
decay/AssignAssigndecaydecay/initial_value*
validate_shape(*
_class

loc:@decay*
use_locking(*
T0
@

decay/readIdentitydecay*
_class

loc:@decay*
T0
9
Q_sample_weightsPlaceholder*
dtype0*
shape: 
1
Q_targetPlaceholder*
dtype0*
shape: 
$
subSubQ/addQ_target*
T0

SquareSquaresub*
T0
@
Mean/reduction_indicesConst*
dtype0*
value	B :
R
MeanMeanSquareMean/reduction_indices*
T0*
	keep_dims( *

Tidx0
A
Mean_1/reduction_indicesConst*
dtype0*
valueB 
T
Mean_1MeanMeanMean_1/reduction_indices*
T0*
	keep_dims( *

Tidx0
-
mulMulMean_1Q_sample_weights*
T0
7

NotEqual/yConst*
dtype0*
valueB
 *    
;
NotEqualNotEqualQ_sample_weights
NotEqual/y*
T0
.
CastCastNotEqual*

DstT0*

SrcT0

3
ConstConst*
dtype0*
valueB: 
A
Mean_2MeanCastConst*
T0*
	keep_dims( *

Tidx0
 
divDivmulMean_2*
T0
5
Const_1Const*
dtype0*
valueB: 
B
Mean_3MeandivConst_1*
T0*
	keep_dims( *

Tidx0
4
mul_1/xConst*
dtype0*
valueB
 *  ?
&
mul_1Mulmul_1/xMean_3*
T0
R
gradients/ShapeConst*
dtype0*
_class

loc:@mul_1*
valueB 
V
gradients/ConstConst*
dtype0*
_class

loc:@mul_1*
valueB
 *  ?
[
gradients/FillFillgradients/Shapegradients/Const*
_class

loc:@mul_1*
T0
]
gradients/mul_1_grad/ShapeConst*
dtype0*
_class

loc:@mul_1*
valueB 
_
gradients/mul_1_grad/Shape_1Const*
dtype0*
_class

loc:@mul_1*
valueB 
 
*gradients/mul_1_grad/BroadcastGradientArgsBroadcastGradientArgsgradients/mul_1_grad/Shapegradients/mul_1_grad/Shape_1*
_class

loc:@mul_1*
T0
Z
gradients/mul_1_grad/mulMulgradients/FillMean_3*
_class

loc:@mul_1*
T0
¥
gradients/mul_1_grad/SumSumgradients/mul_1_grad/mul*gradients/mul_1_grad/BroadcastGradientArgs*
_class

loc:@mul_1*
T0*
	keep_dims( *

Tidx0

gradients/mul_1_grad/ReshapeReshapegradients/mul_1_grad/Sumgradients/mul_1_grad/Shape*
_class

loc:@mul_1*
T0*
Tshape0
]
gradients/mul_1_grad/mul_1Mulmul_1/xgradients/Fill*
_class

loc:@mul_1*
T0
«
gradients/mul_1_grad/Sum_1Sumgradients/mul_1_grad/mul_1,gradients/mul_1_grad/BroadcastGradientArgs:1*
_class

loc:@mul_1*
T0*
	keep_dims( *

Tidx0

gradients/mul_1_grad/Reshape_1Reshapegradients/mul_1_grad/Sum_1gradients/mul_1_grad/Shape_1*
_class

loc:@mul_1*
T0*
Tshape0
l
#gradients/Mean_3_grad/Reshape/shapeConst*
dtype0*
_class
loc:@Mean_3*
valueB:

gradients/Mean_3_grad/ReshapeReshapegradients/mul_1_grad/Reshape_1#gradients/Mean_3_grad/Reshape/shape*
_class
loc:@Mean_3*
T0*
Tshape0
]
gradients/Mean_3_grad/ShapeShapediv*
out_type0*
T0*
_class
loc:@Mean_3

gradients/Mean_3_grad/TileTilegradients/Mean_3_grad/Reshapegradients/Mean_3_grad/Shape*

Tmultiples0*
_class
loc:@Mean_3*
T0
_
gradients/Mean_3_grad/Shape_1Shapediv*
out_type0*
T0*
_class
loc:@Mean_3
a
gradients/Mean_3_grad/Shape_2Const*
dtype0*
_class
loc:@Mean_3*
valueB 
d
gradients/Mean_3_grad/ConstConst*
dtype0*
_class
loc:@Mean_3*
valueB: 

gradients/Mean_3_grad/ProdProdgradients/Mean_3_grad/Shape_1gradients/Mean_3_grad/Const*
_class
loc:@Mean_3*
T0*
	keep_dims( *

Tidx0
f
gradients/Mean_3_grad/Const_1Const*
dtype0*
_class
loc:@Mean_3*
valueB: 
£
gradients/Mean_3_grad/Prod_1Prodgradients/Mean_3_grad/Shape_2gradients/Mean_3_grad/Const_1*
_class
loc:@Mean_3*
T0*
	keep_dims( *

Tidx0
d
gradients/Mean_3_grad/Maximum/yConst*
dtype0*
_class
loc:@Mean_3*
value	B :

gradients/Mean_3_grad/MaximumMaximumgradients/Mean_3_grad/Prod_1gradients/Mean_3_grad/Maximum/y*
_class
loc:@Mean_3*
T0

gradients/Mean_3_grad/floordivDivgradients/Mean_3_grad/Prodgradients/Mean_3_grad/Maximum*
_class
loc:@Mean_3*
T0
u
gradients/Mean_3_grad/CastCastgradients/Mean_3_grad/floordiv*

DstT0*
_class
loc:@Mean_3*

SrcT0

gradients/Mean_3_grad/truedivDivgradients/Mean_3_grad/Tilegradients/Mean_3_grad/Cast*
_class
loc:@Mean_3*
T0
W
gradients/div_grad/ShapeShapemul*
out_type0*
T0*
_class

loc:@div
[
gradients/div_grad/Shape_1Const*
dtype0*
_class

loc:@div*
valueB 

(gradients/div_grad/BroadcastGradientArgsBroadcastGradientArgsgradients/div_grad/Shapegradients/div_grad/Shape_1*
_class

loc:@div*
T0
i
gradients/div_grad/truedivDivgradients/Mean_3_grad/truedivMean_2*
_class

loc:@div*
T0
¡
gradients/div_grad/SumSumgradients/div_grad/truediv(gradients/div_grad/BroadcastGradientArgs*
_class

loc:@div*
T0*
	keep_dims( *

Tidx0

gradients/div_grad/ReshapeReshapegradients/div_grad/Sumgradients/div_grad/Shape*
_class

loc:@div*
T0*
Tshape0
C
gradients/div_grad/NegNegmul*
_class

loc:@div*
T0
L
gradients/div_grad/SquareSquareMean_2*
_class

loc:@div*
T0
w
gradients/div_grad/truediv_1Divgradients/div_grad/Neggradients/div_grad/Square*
_class

loc:@div*
T0
{
gradients/div_grad/mulMulgradients/Mean_3_grad/truedivgradients/div_grad/truediv_1*
_class

loc:@div*
T0
¡
gradients/div_grad/Sum_1Sumgradients/div_grad/mul*gradients/div_grad/BroadcastGradientArgs:1*
_class

loc:@div*
T0*
	keep_dims( *

Tidx0

gradients/div_grad/Reshape_1Reshapegradients/div_grad/Sum_1gradients/div_grad/Shape_1*
_class

loc:@div*
T0*
Tshape0
Z
gradients/mul_grad/ShapeShapeMean_1*
out_type0*
T0*
_class

loc:@mul
f
gradients/mul_grad/Shape_1ShapeQ_sample_weights*
out_type0*
T0*
_class

loc:@mul

(gradients/mul_grad/BroadcastGradientArgsBroadcastGradientArgsgradients/mul_grad/Shapegradients/mul_grad/Shape_1*
_class

loc:@mul*
T0
l
gradients/mul_grad/mulMulgradients/div_grad/ReshapeQ_sample_weights*
_class

loc:@mul*
T0

gradients/mul_grad/SumSumgradients/mul_grad/mul(gradients/mul_grad/BroadcastGradientArgs*
_class

loc:@mul*
T0*
	keep_dims( *

Tidx0

gradients/mul_grad/ReshapeReshapegradients/mul_grad/Sumgradients/mul_grad/Shape*
_class

loc:@mul*
T0*
Tshape0
d
gradients/mul_grad/mul_1MulMean_1gradients/div_grad/Reshape*
_class

loc:@mul*
T0
£
gradients/mul_grad/Sum_1Sumgradients/mul_grad/mul_1*gradients/mul_grad/BroadcastGradientArgs:1*
_class

loc:@mul*
T0*
	keep_dims( *

Tidx0

gradients/mul_grad/Reshape_1Reshapegradients/mul_grad/Sum_1gradients/mul_grad/Shape_1*
_class

loc:@mul*
T0*
Tshape0
^
gradients/Mean_1_grad/ShapeShapeMean*
out_type0*
T0*
_class
loc:@Mean_1
_
gradients/Mean_1_grad/SizeConst*
dtype0*
_class
loc:@Mean_1*
value	B :
z
gradients/Mean_1_grad/addAddMean_1/reduction_indicesgradients/Mean_1_grad/Size*
_class
loc:@Mean_1*
T0
{
gradients/Mean_1_grad/modModgradients/Mean_1_grad/addgradients/Mean_1_grad/Size*
_class
loc:@Mean_1*
T0
f
gradients/Mean_1_grad/Shape_1Const*
dtype0*
_class
loc:@Mean_1*
valueB: 
f
!gradients/Mean_1_grad/range/startConst*
dtype0*
_class
loc:@Mean_1*
value	B : 
f
!gradients/Mean_1_grad/range/deltaConst*
dtype0*
_class
loc:@Mean_1*
value	B :
­
gradients/Mean_1_grad/rangeRange!gradients/Mean_1_grad/range/startgradients/Mean_1_grad/Size!gradients/Mean_1_grad/range/delta*
_class
loc:@Mean_1*

Tidx0
e
 gradients/Mean_1_grad/Fill/valueConst*
dtype0*
_class
loc:@Mean_1*
value	B :

gradients/Mean_1_grad/FillFillgradients/Mean_1_grad/Shape_1 gradients/Mean_1_grad/Fill/value*
_class
loc:@Mean_1*
T0
Ò
#gradients/Mean_1_grad/DynamicStitchDynamicStitchgradients/Mean_1_grad/rangegradients/Mean_1_grad/modgradients/Mean_1_grad/Shapegradients/Mean_1_grad/Fill*
_class
loc:@Mean_1*
T0*
N
d
gradients/Mean_1_grad/Maximum/yConst*
dtype0*
_class
loc:@Mean_1*
value	B :

gradients/Mean_1_grad/MaximumMaximum#gradients/Mean_1_grad/DynamicStitchgradients/Mean_1_grad/Maximum/y*
_class
loc:@Mean_1*
T0

gradients/Mean_1_grad/floordivDivgradients/Mean_1_grad/Shapegradients/Mean_1_grad/Maximum*
_class
loc:@Mean_1*
T0

gradients/Mean_1_grad/ReshapeReshapegradients/mul_grad/Reshape#gradients/Mean_1_grad/DynamicStitch*
_class
loc:@Mean_1*
T0*
Tshape0

gradients/Mean_1_grad/TileTilegradients/Mean_1_grad/Reshapegradients/Mean_1_grad/floordiv*

Tmultiples0*
_class
loc:@Mean_1*
T0
`
gradients/Mean_1_grad/Shape_2ShapeMean*
out_type0*
T0*
_class
loc:@Mean_1
b
gradients/Mean_1_grad/Shape_3ShapeMean_1*
out_type0*
T0*
_class
loc:@Mean_1
d
gradients/Mean_1_grad/ConstConst*
dtype0*
_class
loc:@Mean_1*
valueB: 

gradients/Mean_1_grad/ProdProdgradients/Mean_1_grad/Shape_2gradients/Mean_1_grad/Const*
_class
loc:@Mean_1*
T0*
	keep_dims( *

Tidx0
f
gradients/Mean_1_grad/Const_1Const*
dtype0*
_class
loc:@Mean_1*
valueB: 
£
gradients/Mean_1_grad/Prod_1Prodgradients/Mean_1_grad/Shape_3gradients/Mean_1_grad/Const_1*
_class
loc:@Mean_1*
T0*
	keep_dims( *

Tidx0
f
!gradients/Mean_1_grad/Maximum_1/yConst*
dtype0*
_class
loc:@Mean_1*
value	B :

gradients/Mean_1_grad/Maximum_1Maximumgradients/Mean_1_grad/Prod_1!gradients/Mean_1_grad/Maximum_1/y*
_class
loc:@Mean_1*
T0

 gradients/Mean_1_grad/floordiv_1Divgradients/Mean_1_grad/Prodgradients/Mean_1_grad/Maximum_1*
_class
loc:@Mean_1*
T0
w
gradients/Mean_1_grad/CastCast gradients/Mean_1_grad/floordiv_1*

DstT0*
_class
loc:@Mean_1*

SrcT0

gradients/Mean_1_grad/truedivDivgradients/Mean_1_grad/Tilegradients/Mean_1_grad/Cast*
_class
loc:@Mean_1*
T0
\
gradients/Mean_grad/ShapeShapeSquare*
out_type0*
T0*
_class
	loc:@Mean
[
gradients/Mean_grad/SizeConst*
dtype0*
_class
	loc:@Mean*
value	B :
r
gradients/Mean_grad/addAddMean/reduction_indicesgradients/Mean_grad/Size*
_class
	loc:@Mean*
T0
s
gradients/Mean_grad/modModgradients/Mean_grad/addgradients/Mean_grad/Size*
_class
	loc:@Mean*
T0
]
gradients/Mean_grad/Shape_1Const*
dtype0*
_class
	loc:@Mean*
valueB 
b
gradients/Mean_grad/range/startConst*
dtype0*
_class
	loc:@Mean*
value	B : 
b
gradients/Mean_grad/range/deltaConst*
dtype0*
_class
	loc:@Mean*
value	B :
£
gradients/Mean_grad/rangeRangegradients/Mean_grad/range/startgradients/Mean_grad/Sizegradients/Mean_grad/range/delta*
_class
	loc:@Mean*

Tidx0
a
gradients/Mean_grad/Fill/valueConst*
dtype0*
_class
	loc:@Mean*
value	B :

gradients/Mean_grad/FillFillgradients/Mean_grad/Shape_1gradients/Mean_grad/Fill/value*
_class
	loc:@Mean*
T0
Æ
!gradients/Mean_grad/DynamicStitchDynamicStitchgradients/Mean_grad/rangegradients/Mean_grad/modgradients/Mean_grad/Shapegradients/Mean_grad/Fill*
_class
	loc:@Mean*
T0*
N
`
gradients/Mean_grad/Maximum/yConst*
dtype0*
_class
	loc:@Mean*
value	B :

gradients/Mean_grad/MaximumMaximum!gradients/Mean_grad/DynamicStitchgradients/Mean_grad/Maximum/y*
_class
	loc:@Mean*
T0
}
gradients/Mean_grad/floordivDivgradients/Mean_grad/Shapegradients/Mean_grad/Maximum*
_class
	loc:@Mean*
T0

gradients/Mean_grad/ReshapeReshapegradients/Mean_1_grad/truediv!gradients/Mean_grad/DynamicStitch*
_class
	loc:@Mean*
T0*
Tshape0

gradients/Mean_grad/TileTilegradients/Mean_grad/Reshapegradients/Mean_grad/floordiv*

Tmultiples0*
_class
	loc:@Mean*
T0
^
gradients/Mean_grad/Shape_2ShapeSquare*
out_type0*
T0*
_class
	loc:@Mean
\
gradients/Mean_grad/Shape_3ShapeMean*
out_type0*
T0*
_class
	loc:@Mean
`
gradients/Mean_grad/ConstConst*
dtype0*
_class
	loc:@Mean*
valueB: 

gradients/Mean_grad/ProdProdgradients/Mean_grad/Shape_2gradients/Mean_grad/Const*
_class
	loc:@Mean*
T0*
	keep_dims( *

Tidx0
b
gradients/Mean_grad/Const_1Const*
dtype0*
_class
	loc:@Mean*
valueB: 

gradients/Mean_grad/Prod_1Prodgradients/Mean_grad/Shape_3gradients/Mean_grad/Const_1*
_class
	loc:@Mean*
T0*
	keep_dims( *

Tidx0
b
gradients/Mean_grad/Maximum_1/yConst*
dtype0*
_class
	loc:@Mean*
value	B :

gradients/Mean_grad/Maximum_1Maximumgradients/Mean_grad/Prod_1gradients/Mean_grad/Maximum_1/y*
_class
	loc:@Mean*
T0

gradients/Mean_grad/floordiv_1Divgradients/Mean_grad/Prodgradients/Mean_grad/Maximum_1*
_class
	loc:@Mean*
T0
q
gradients/Mean_grad/CastCastgradients/Mean_grad/floordiv_1*

DstT0*
_class
	loc:@Mean*

SrcT0
x
gradients/Mean_grad/truedivDivgradients/Mean_grad/Tilegradients/Mean_grad/Cast*
_class
	loc:@Mean*
T0

gradients/Square_grad/mul/xConst^gradients/Mean_grad/truediv*
dtype0*
_class
loc:@Square*
valueB
 *   @
f
gradients/Square_grad/mulMulgradients/Square_grad/mul/xsub*
_class
loc:@Square*
T0
~
gradients/Square_grad/mul_1Mulgradients/Mean_grad/truedivgradients/Square_grad/mul*
_class
loc:@Square*
T0
Y
gradients/sub_grad/ShapeShapeQ/add*
out_type0*
T0*
_class

loc:@sub
^
gradients/sub_grad/Shape_1ShapeQ_target*
out_type0*
T0*
_class

loc:@sub

(gradients/sub_grad/BroadcastGradientArgsBroadcastGradientArgsgradients/sub_grad/Shapegradients/sub_grad/Shape_1*
_class

loc:@sub*
T0
¢
gradients/sub_grad/SumSumgradients/Square_grad/mul_1(gradients/sub_grad/BroadcastGradientArgs*
_class

loc:@sub*
T0*
	keep_dims( *

Tidx0

gradients/sub_grad/ReshapeReshapegradients/sub_grad/Sumgradients/sub_grad/Shape*
_class

loc:@sub*
T0*
Tshape0
¦
gradients/sub_grad/Sum_1Sumgradients/Square_grad/mul_1*gradients/sub_grad/BroadcastGradientArgs:1*
_class

loc:@sub*
T0*
	keep_dims( *

Tidx0
X
gradients/sub_grad/NegNeggradients/sub_grad/Sum_1*
_class

loc:@sub*
T0

gradients/sub_grad/Reshape_1Reshapegradients/sub_grad/Neggradients/sub_grad/Shape_1*
_class

loc:@sub*
T0*
Tshape0
a
gradients/Q/add_grad/ShapeShape	V/BiasAdd*
out_type0*
T0*
_class

loc:@Q/add
_
gradients/Q/add_grad/Shape_1ShapeA/mul*
out_type0*
T0*
_class

loc:@Q/add
 
*gradients/Q/add_grad/BroadcastGradientArgsBroadcastGradientArgsgradients/Q/add_grad/Shapegradients/Q/add_grad/Shape_1*
_class

loc:@Q/add*
T0
§
gradients/Q/add_grad/SumSumgradients/sub_grad/Reshape*gradients/Q/add_grad/BroadcastGradientArgs*
_class

loc:@Q/add*
T0*
	keep_dims( *

Tidx0

gradients/Q/add_grad/ReshapeReshapegradients/Q/add_grad/Sumgradients/Q/add_grad/Shape*
_class

loc:@Q/add*
T0*
Tshape0
«
gradients/Q/add_grad/Sum_1Sumgradients/sub_grad/Reshape,gradients/Q/add_grad/BroadcastGradientArgs:1*
_class

loc:@Q/add*
T0*
	keep_dims( *

Tidx0

gradients/Q/add_grad/Reshape_1Reshapegradients/Q/add_grad/Sum_1gradients/Q/add_grad/Shape_1*
_class

loc:@Q/add*
T0*
Tshape0

$gradients/V/BiasAdd_grad/BiasAddGradBiasAddGradgradients/Q/add_grad/Reshape*
_class
loc:@V/BiasAdd*
T0*
data_formatNHWC
`
gradients/A/mul_grad/ShapeShapeP/Square*
out_type0*
T0*
_class

loc:@A/mul
f
gradients/A/mul_grad/Shape_1Shapelambda_2/mul*
out_type0*
T0*
_class

loc:@A/mul
 
*gradients/A/mul_grad/BroadcastGradientArgsBroadcastGradientArgsgradients/A/mul_grad/Shapegradients/A/mul_grad/Shape_1*
_class

loc:@A/mul*
T0
p
gradients/A/mul_grad/mulMulgradients/Q/add_grad/Reshape_1lambda_2/mul*
_class

loc:@A/mul*
T0
¥
gradients/A/mul_grad/SumSumgradients/A/mul_grad/mul*gradients/A/mul_grad/BroadcastGradientArgs*
_class

loc:@A/mul*
T0*
	keep_dims( *

Tidx0

gradients/A/mul_grad/ReshapeReshapegradients/A/mul_grad/Sumgradients/A/mul_grad/Shape*
_class

loc:@A/mul*
T0*
Tshape0
n
gradients/A/mul_grad/mul_1MulP/Squaregradients/Q/add_grad/Reshape_1*
_class

loc:@A/mul*
T0
«
gradients/A/mul_grad/Sum_1Sumgradients/A/mul_grad/mul_1,gradients/A/mul_grad/BroadcastGradientArgs:1*
_class

loc:@A/mul*
T0*
	keep_dims( *

Tidx0

gradients/A/mul_grad/Reshape_1Reshapegradients/A/mul_grad/Sum_1gradients/A/mul_grad/Shape_1*
_class

loc:@A/mul*
T0*
Tshape0
¡
gradients/V/MatMul_grad/MatMulMatMulgradients/Q/add_grad/ReshapeV/kernel/read*
transpose_b(*
transpose_a( *
_class
loc:@V/MatMul*
T0
¶
 gradients/V/MatMul_grad/MatMul_1MatMul batch_normalization_3/cond/Mergegradients/Q/add_grad/Reshape*
transpose_b( *
transpose_a(*
_class
loc:@V/MatMul*
T0

gradients/P/Square_grad/mul/xConst^gradients/A/mul_grad/Reshape*
dtype0*
_class
loc:@P/Square*
valueB
 *   @
n
gradients/P/Square_grad/mulMulgradients/P/Square_grad/mul/xP/Exp*
_class
loc:@P/Square*
T0

gradients/P/Square_grad/mul_1Mulgradients/A/mul_grad/Reshapegradients/P/Square_grad/mul*
_class
loc:@P/Square*
T0
k
!gradients/lambda_2/mul_grad/ShapeConst*
dtype0*
_class
loc:@lambda_2/mul*
valueB 
w
#gradients/lambda_2/mul_grad/Shape_1Shapelambda_2/Square*
out_type0*
T0*
_class
loc:@lambda_2/mul
¼
1gradients/lambda_2/mul_grad/BroadcastGradientArgsBroadcastGradientArgs!gradients/lambda_2/mul_grad/Shape#gradients/lambda_2/mul_grad/Shape_1*
_class
loc:@lambda_2/mul*
T0

gradients/lambda_2/mul_grad/mulMulgradients/A/mul_grad/Reshape_1lambda_2/Square*
_class
loc:@lambda_2/mul*
T0
Á
gradients/lambda_2/mul_grad/SumSumgradients/lambda_2/mul_grad/mul1gradients/lambda_2/mul_grad/BroadcastGradientArgs*
_class
loc:@lambda_2/mul*
T0*
	keep_dims( *

Tidx0
ª
#gradients/lambda_2/mul_grad/ReshapeReshapegradients/lambda_2/mul_grad/Sum!gradients/lambda_2/mul_grad/Shape*
_class
loc:@lambda_2/mul*
T0*
Tshape0

!gradients/lambda_2/mul_grad/mul_1Mullambda_2/mul/xgradients/A/mul_grad/Reshape_1*
_class
loc:@lambda_2/mul*
T0
Ç
!gradients/lambda_2/mul_grad/Sum_1Sum!gradients/lambda_2/mul_grad/mul_13gradients/lambda_2/mul_grad/BroadcastGradientArgs:1*
_class
loc:@lambda_2/mul*
T0*
	keep_dims( *

Tidx0
°
%gradients/lambda_2/mul_grad/Reshape_1Reshape!gradients/lambda_2/mul_grad/Sum_1#gradients/lambda_2/mul_grad/Shape_1*
_class
loc:@lambda_2/mul*
T0*
Tshape0
h
gradients/P/Exp_grad/mulMulgradients/P/Square_grad/mul_1P/Exp*
_class

loc:@P/Exp*
T0

$gradients/lambda_2/Square_grad/mul/xConst&^gradients/lambda_2/mul_grad/Reshape_1*
dtype0*"
_class
loc:@lambda_2/Square*
valueB
 *   @

"gradients/lambda_2/Square_grad/mulMul$gradients/lambda_2/Square_grad/mul/x	add_1/add*"
_class
loc:@lambda_2/Square*
T0
£
$gradients/lambda_2/Square_grad/mul_1Mul%gradients/lambda_2/mul_grad/Reshape_1"gradients/lambda_2/Square_grad/mul*"
_class
loc:@lambda_2/Square*
T0

*gradients/dense_1/BiasAdd_grad/BiasAddGradBiasAddGradgradients/P/Exp_grad/mul*"
_class
loc:@dense_1/BiasAdd*
T0*
data_formatNHWC
o
gradients/add_1/add_grad/ShapeShapea/strided_slice*
out_type0*
T0*
_class
loc:@add_1/add
n
 gradients/add_1/add_grad/Shape_1Shapelambda_1/Neg*
out_type0*
T0*
_class
loc:@add_1/add
°
.gradients/add_1/add_grad/BroadcastGradientArgsBroadcastGradientArgsgradients/add_1/add_grad/Shape gradients/add_1/add_grad/Shape_1*
_class
loc:@add_1/add*
T0
½
gradients/add_1/add_grad/SumSum$gradients/lambda_2/Square_grad/mul_1.gradients/add_1/add_grad/BroadcastGradientArgs*
_class
loc:@add_1/add*
T0*
	keep_dims( *

Tidx0

 gradients/add_1/add_grad/ReshapeReshapegradients/add_1/add_grad/Sumgradients/add_1/add_grad/Shape*
_class
loc:@add_1/add*
T0*
Tshape0
Á
gradients/add_1/add_grad/Sum_1Sum$gradients/lambda_2/Square_grad/mul_10gradients/add_1/add_grad/BroadcastGradientArgs:1*
_class
loc:@add_1/add*
T0*
	keep_dims( *

Tidx0
¤
"gradients/add_1/add_grad/Reshape_1Reshapegradients/add_1/add_grad/Sum_1 gradients/add_1/add_grad/Shape_1*
_class
loc:@add_1/add*
T0*
Tshape0
¯
$gradients/dense_1/MatMul_grad/MatMulMatMulgradients/P/Exp_grad/muldense_1/kernel/read*
transpose_b(*
transpose_a( *!
_class
loc:@dense_1/MatMul*
T0
¾
&gradients/dense_1/MatMul_grad/MatMul_1MatMul batch_normalization_3/cond/Mergegradients/P/Exp_grad/mul*
transpose_b( *
transpose_a(*!
_class
loc:@dense_1/MatMul*
T0
t
gradients/lambda_1/Neg_grad/NegNeg"gradients/add_1/add_grad/Reshape_1*
_class
loc:@lambda_1/Neg*
T0
_
gradients/mu/mul_grad/ShapeConst*
dtype0*
_class
loc:@mu/mul*
valueB 
h
gradients/mu/mul_grad/Shape_1Shapedense_2/Tanh*
out_type0*
T0*
_class
loc:@mu/mul
¤
+gradients/mu/mul_grad/BroadcastGradientArgsBroadcastGradientArgsgradients/mu/mul_grad/Shapegradients/mu/mul_grad/Shape_1*
_class
loc:@mu/mul*
T0
s
gradients/mu/mul_grad/mulMulgradients/lambda_1/Neg_grad/Negdense_2/Tanh*
_class
loc:@mu/mul*
T0
©
gradients/mu/mul_grad/SumSumgradients/mu/mul_grad/mul+gradients/mu/mul_grad/BroadcastGradientArgs*
_class
loc:@mu/mul*
T0*
	keep_dims( *

Tidx0

gradients/mu/mul_grad/ReshapeReshapegradients/mu/mul_grad/Sumgradients/mu/mul_grad/Shape*
_class
loc:@mu/mul*
T0*
Tshape0
q
gradients/mu/mul_grad/mul_1Mulmu/mul/xgradients/lambda_1/Neg_grad/Neg*
_class
loc:@mu/mul*
T0
¯
gradients/mu/mul_grad/Sum_1Sumgradients/mu/mul_grad/mul_1-gradients/mu/mul_grad/BroadcastGradientArgs:1*
_class
loc:@mu/mul*
T0*
	keep_dims( *

Tidx0

gradients/mu/mul_grad/Reshape_1Reshapegradients/mu/mul_grad/Sum_1gradients/mu/mul_grad/Shape_1*
_class
loc:@mu/mul*
T0*
Tshape0

$gradients/dense_2/Tanh_grad/TanhGradTanhGraddense_2/Tanhgradients/mu/mul_grad/Reshape_1*
_class
loc:@dense_2/Tanh*
T0
£
*gradients/dense_2/BiasAdd_grad/BiasAddGradBiasAddGrad$gradients/dense_2/Tanh_grad/TanhGrad*"
_class
loc:@dense_2/BiasAdd*
T0*
data_formatNHWC
»
$gradients/dense_2/MatMul_grad/MatMulMatMul$gradients/dense_2/Tanh_grad/TanhGraddense_2/kernel/read*
transpose_b(*
transpose_a( *!
_class
loc:@dense_2/MatMul*
T0
Ê
&gradients/dense_2/MatMul_grad/MatMul_1MatMul batch_normalization_3/cond/Merge$gradients/dense_2/Tanh_grad/TanhGrad*
transpose_b( *
transpose_a(*!
_class
loc:@dense_2/MatMul*
T0
±
gradients/AddNAddNgradients/V/MatMul_grad/MatMul$gradients/dense_1/MatMul_grad/MatMul$gradients/dense_2/MatMul_grad/MatMul*
_class
loc:@V/MatMul*
T0*
N

9gradients/batch_normalization_3/cond/Merge_grad/cond_gradSwitchgradients/AddN"batch_normalization_3/cond/pred_id*
_class
loc:@V/MatMul*
T0
Ì
?gradients/batch_normalization_3/cond/batchnorm/add_1_grad/ShapeShape*batch_normalization_3/cond/batchnorm/mul_1*
out_type0*
T0*=
_class3
1/loc:@batch_normalization_3/cond/batchnorm/add_1
®
Agradients/batch_normalization_3/cond/batchnorm/add_1_grad/Shape_1Const*
dtype0*=
_class3
1/loc:@batch_normalization_3/cond/batchnorm/add_1*
valueB:d
´
Ogradients/batch_normalization_3/cond/batchnorm/add_1_grad/BroadcastGradientArgsBroadcastGradientArgs?gradients/batch_normalization_3/cond/batchnorm/add_1_grad/ShapeAgradients/batch_normalization_3/cond/batchnorm/add_1_grad/Shape_1*=
_class3
1/loc:@batch_normalization_3/cond/batchnorm/add_1*
T0
µ
=gradients/batch_normalization_3/cond/batchnorm/add_1_grad/SumSum9gradients/batch_normalization_3/cond/Merge_grad/cond_gradOgradients/batch_normalization_3/cond/batchnorm/add_1_grad/BroadcastGradientArgs*=
_class3
1/loc:@batch_normalization_3/cond/batchnorm/add_1*
T0*
	keep_dims( *

Tidx0
¢
Agradients/batch_normalization_3/cond/batchnorm/add_1_grad/ReshapeReshape=gradients/batch_normalization_3/cond/batchnorm/add_1_grad/Sum?gradients/batch_normalization_3/cond/batchnorm/add_1_grad/Shape*=
_class3
1/loc:@batch_normalization_3/cond/batchnorm/add_1*
T0*
Tshape0
¹
?gradients/batch_normalization_3/cond/batchnorm/add_1_grad/Sum_1Sum9gradients/batch_normalization_3/cond/Merge_grad/cond_gradQgradients/batch_normalization_3/cond/batchnorm/add_1_grad/BroadcastGradientArgs:1*=
_class3
1/loc:@batch_normalization_3/cond/batchnorm/add_1*
T0*
	keep_dims( *

Tidx0
¨
Cgradients/batch_normalization_3/cond/batchnorm/add_1_grad/Reshape_1Reshape?gradients/batch_normalization_3/cond/batchnorm/add_1_grad/Sum_1Agradients/batch_normalization_3/cond/batchnorm/add_1_grad/Shape_1*=
_class3
1/loc:@batch_normalization_3/cond/batchnorm/add_1*
T0*
Tshape0
¨
gradients/SwitchSwitch%batch_normalization_3/batchnorm/add_1"batch_normalization_3/cond/pred_id*8
_class.
,*loc:@batch_normalization_3/batchnorm/add_1*
T0

gradients/Shape_1Shapegradients/Switch*
out_type0*
T0*8
_class.
,*loc:@batch_normalization_3/batchnorm/add_1
|
gradients/zeros/ConstConst*
dtype0*8
_class.
,*loc:@batch_normalization_3/batchnorm/add_1*
valueB
 *    

gradients/zerosFillgradients/Shape_1gradients/zeros/Const*8
_class.
,*loc:@batch_normalization_3/batchnorm/add_1*
T0
ß
<gradients/batch_normalization_3/cond/Switch_1_grad/cond_gradMerge;gradients/batch_normalization_3/cond/Merge_grad/cond_grad:1gradients/zeros*8
_class.
,*loc:@batch_normalization_3/batchnorm/add_1*
T0*
N
Ó
?gradients/batch_normalization_3/cond/batchnorm/mul_1_grad/ShapeShape1batch_normalization_3/cond/batchnorm/mul_1/Switch*
out_type0*
T0*=
_class3
1/loc:@batch_normalization_3/cond/batchnorm/mul_1
®
Agradients/batch_normalization_3/cond/batchnorm/mul_1_grad/Shape_1Const*
dtype0*=
_class3
1/loc:@batch_normalization_3/cond/batchnorm/mul_1*
valueB:d
´
Ogradients/batch_normalization_3/cond/batchnorm/mul_1_grad/BroadcastGradientArgsBroadcastGradientArgs?gradients/batch_normalization_3/cond/batchnorm/mul_1_grad/ShapeAgradients/batch_normalization_3/cond/batchnorm/mul_1_grad/Shape_1*=
_class3
1/loc:@batch_normalization_3/cond/batchnorm/mul_1*
T0
ù
=gradients/batch_normalization_3/cond/batchnorm/mul_1_grad/mulMulAgradients/batch_normalization_3/cond/batchnorm/add_1_grad/Reshape(batch_normalization_3/cond/batchnorm/mul*=
_class3
1/loc:@batch_normalization_3/cond/batchnorm/mul_1*
T0
¹
=gradients/batch_normalization_3/cond/batchnorm/mul_1_grad/SumSum=gradients/batch_normalization_3/cond/batchnorm/mul_1_grad/mulOgradients/batch_normalization_3/cond/batchnorm/mul_1_grad/BroadcastGradientArgs*=
_class3
1/loc:@batch_normalization_3/cond/batchnorm/mul_1*
T0*
	keep_dims( *

Tidx0
¢
Agradients/batch_normalization_3/cond/batchnorm/mul_1_grad/ReshapeReshape=gradients/batch_normalization_3/cond/batchnorm/mul_1_grad/Sum?gradients/batch_normalization_3/cond/batchnorm/mul_1_grad/Shape*=
_class3
1/loc:@batch_normalization_3/cond/batchnorm/mul_1*
T0*
Tshape0

?gradients/batch_normalization_3/cond/batchnorm/mul_1_grad/mul_1Mul1batch_normalization_3/cond/batchnorm/mul_1/SwitchAgradients/batch_normalization_3/cond/batchnorm/add_1_grad/Reshape*=
_class3
1/loc:@batch_normalization_3/cond/batchnorm/mul_1*
T0
¿
?gradients/batch_normalization_3/cond/batchnorm/mul_1_grad/Sum_1Sum?gradients/batch_normalization_3/cond/batchnorm/mul_1_grad/mul_1Qgradients/batch_normalization_3/cond/batchnorm/mul_1_grad/BroadcastGradientArgs:1*=
_class3
1/loc:@batch_normalization_3/cond/batchnorm/mul_1*
T0*
	keep_dims( *

Tidx0
¨
Cgradients/batch_normalization_3/cond/batchnorm/mul_1_grad/Reshape_1Reshape?gradients/batch_normalization_3/cond/batchnorm/mul_1_grad/Sum_1Agradients/batch_normalization_3/cond/batchnorm/mul_1_grad/Shape_1*=
_class3
1/loc:@batch_normalization_3/cond/batchnorm/mul_1*
T0*
Tshape0
¨
=gradients/batch_normalization_3/cond/batchnorm/sub_grad/ShapeConst*
dtype0*;
_class1
/-loc:@batch_normalization_3/cond/batchnorm/sub*
valueB:d
ª
?gradients/batch_normalization_3/cond/batchnorm/sub_grad/Shape_1Const*
dtype0*;
_class1
/-loc:@batch_normalization_3/cond/batchnorm/sub*
valueB:d
¬
Mgradients/batch_normalization_3/cond/batchnorm/sub_grad/BroadcastGradientArgsBroadcastGradientArgs=gradients/batch_normalization_3/cond/batchnorm/sub_grad/Shape?gradients/batch_normalization_3/cond/batchnorm/sub_grad/Shape_1*;
_class1
/-loc:@batch_normalization_3/cond/batchnorm/sub*
T0
¹
;gradients/batch_normalization_3/cond/batchnorm/sub_grad/SumSumCgradients/batch_normalization_3/cond/batchnorm/add_1_grad/Reshape_1Mgradients/batch_normalization_3/cond/batchnorm/sub_grad/BroadcastGradientArgs*;
_class1
/-loc:@batch_normalization_3/cond/batchnorm/sub*
T0*
	keep_dims( *

Tidx0

?gradients/batch_normalization_3/cond/batchnorm/sub_grad/ReshapeReshape;gradients/batch_normalization_3/cond/batchnorm/sub_grad/Sum=gradients/batch_normalization_3/cond/batchnorm/sub_grad/Shape*;
_class1
/-loc:@batch_normalization_3/cond/batchnorm/sub*
T0*
Tshape0
½
=gradients/batch_normalization_3/cond/batchnorm/sub_grad/Sum_1SumCgradients/batch_normalization_3/cond/batchnorm/add_1_grad/Reshape_1Ogradients/batch_normalization_3/cond/batchnorm/sub_grad/BroadcastGradientArgs:1*;
_class1
/-loc:@batch_normalization_3/cond/batchnorm/sub*
T0*
	keep_dims( *

Tidx0
Ç
;gradients/batch_normalization_3/cond/batchnorm/sub_grad/NegNeg=gradients/batch_normalization_3/cond/batchnorm/sub_grad/Sum_1*;
_class1
/-loc:@batch_normalization_3/cond/batchnorm/sub*
T0

Agradients/batch_normalization_3/cond/batchnorm/sub_grad/Reshape_1Reshape;gradients/batch_normalization_3/cond/batchnorm/sub_grad/Neg?gradients/batch_normalization_3/cond/batchnorm/sub_grad/Shape_1*;
_class1
/-loc:@batch_normalization_3/cond/batchnorm/sub*
T0*
Tshape0
½
:gradients/batch_normalization_3/batchnorm/add_1_grad/ShapeShape%batch_normalization_3/batchnorm/mul_1*
out_type0*
T0*8
_class.
,*loc:@batch_normalization_3/batchnorm/add_1
¤
<gradients/batch_normalization_3/batchnorm/add_1_grad/Shape_1Const*
dtype0*8
_class.
,*loc:@batch_normalization_3/batchnorm/add_1*
valueB:d
 
Jgradients/batch_normalization_3/batchnorm/add_1_grad/BroadcastGradientArgsBroadcastGradientArgs:gradients/batch_normalization_3/batchnorm/add_1_grad/Shape<gradients/batch_normalization_3/batchnorm/add_1_grad/Shape_1*8
_class.
,*loc:@batch_normalization_3/batchnorm/add_1*
T0
©
8gradients/batch_normalization_3/batchnorm/add_1_grad/SumSum<gradients/batch_normalization_3/cond/Switch_1_grad/cond_gradJgradients/batch_normalization_3/batchnorm/add_1_grad/BroadcastGradientArgs*8
_class.
,*loc:@batch_normalization_3/batchnorm/add_1*
T0*
	keep_dims( *

Tidx0

<gradients/batch_normalization_3/batchnorm/add_1_grad/ReshapeReshape8gradients/batch_normalization_3/batchnorm/add_1_grad/Sum:gradients/batch_normalization_3/batchnorm/add_1_grad/Shape*8
_class.
,*loc:@batch_normalization_3/batchnorm/add_1*
T0*
Tshape0
­
:gradients/batch_normalization_3/batchnorm/add_1_grad/Sum_1Sum<gradients/batch_normalization_3/cond/Switch_1_grad/cond_gradLgradients/batch_normalization_3/batchnorm/add_1_grad/BroadcastGradientArgs:1*8
_class.
,*loc:@batch_normalization_3/batchnorm/add_1*
T0*
	keep_dims( *

Tidx0

>gradients/batch_normalization_3/batchnorm/add_1_grad/Reshape_1Reshape:gradients/batch_normalization_3/batchnorm/add_1_grad/Sum_1<gradients/batch_normalization_3/batchnorm/add_1_grad/Shape_1*8
_class.
,*loc:@batch_normalization_3/batchnorm/add_1*
T0*
Tshape0
n
gradients/Switch_1Switchh2/Relu"batch_normalization_3/cond/pred_id*
_class
loc:@h2/Relu*
T0
e
gradients/Shape_2Shapegradients/Switch_1:1*
out_type0*
T0*
_class
loc:@h2/Relu
`
gradients/zeros_1/ConstConst*
dtype0*
_class
loc:@h2/Relu*
valueB
 *    
j
gradients/zeros_1Fillgradients/Shape_2gradients/zeros_1/Const*
_class
loc:@h2/Relu*
T0
×
Jgradients/batch_normalization_3/cond/batchnorm/mul_1/Switch_grad/cond_gradMergeAgradients/batch_normalization_3/cond/batchnorm/mul_1_grad/Reshapegradients/zeros_1*
_class
loc:@h2/Relu*
T0*
N

gradients/Switch_2Switchbatch_normalization_3/beta/read"batch_normalization_3/cond/pred_id*-
_class#
!loc:@batch_normalization_3/beta*
T0
x
gradients/Shape_3Shapegradients/Switch_2:1*
out_type0*
T0*-
_class#
!loc:@batch_normalization_3/beta
s
gradients/zeros_2/ConstConst*
dtype0*-
_class#
!loc:@batch_normalization_3/beta*
valueB
 *    
}
gradients/zeros_2Fillgradients/Shape_3gradients/zeros_2/Const*-
_class#
!loc:@batch_normalization_3/beta*
T0
æ
Hgradients/batch_normalization_3/cond/batchnorm/sub/Switch_grad/cond_gradMerge?gradients/batch_normalization_3/cond/batchnorm/sub_grad/Reshapegradients/zeros_2*-
_class#
!loc:@batch_normalization_3/beta*
T0*
N
¬
?gradients/batch_normalization_3/cond/batchnorm/mul_2_grad/ShapeConst*
dtype0*=
_class3
1/loc:@batch_normalization_3/cond/batchnorm/mul_2*
valueB:d
®
Agradients/batch_normalization_3/cond/batchnorm/mul_2_grad/Shape_1Const*
dtype0*=
_class3
1/loc:@batch_normalization_3/cond/batchnorm/mul_2*
valueB:d
´
Ogradients/batch_normalization_3/cond/batchnorm/mul_2_grad/BroadcastGradientArgsBroadcastGradientArgs?gradients/batch_normalization_3/cond/batchnorm/mul_2_grad/ShapeAgradients/batch_normalization_3/cond/batchnorm/mul_2_grad/Shape_1*=
_class3
1/loc:@batch_normalization_3/cond/batchnorm/mul_2*
T0
ù
=gradients/batch_normalization_3/cond/batchnorm/mul_2_grad/mulMulAgradients/batch_normalization_3/cond/batchnorm/sub_grad/Reshape_1(batch_normalization_3/cond/batchnorm/mul*=
_class3
1/loc:@batch_normalization_3/cond/batchnorm/mul_2*
T0
¹
=gradients/batch_normalization_3/cond/batchnorm/mul_2_grad/SumSum=gradients/batch_normalization_3/cond/batchnorm/mul_2_grad/mulOgradients/batch_normalization_3/cond/batchnorm/mul_2_grad/BroadcastGradientArgs*=
_class3
1/loc:@batch_normalization_3/cond/batchnorm/mul_2*
T0*
	keep_dims( *

Tidx0
¢
Agradients/batch_normalization_3/cond/batchnorm/mul_2_grad/ReshapeReshape=gradients/batch_normalization_3/cond/batchnorm/mul_2_grad/Sum?gradients/batch_normalization_3/cond/batchnorm/mul_2_grad/Shape*=
_class3
1/loc:@batch_normalization_3/cond/batchnorm/mul_2*
T0*
Tshape0

?gradients/batch_normalization_3/cond/batchnorm/mul_2_grad/mul_1Mul1batch_normalization_3/cond/batchnorm/mul_2/SwitchAgradients/batch_normalization_3/cond/batchnorm/sub_grad/Reshape_1*=
_class3
1/loc:@batch_normalization_3/cond/batchnorm/mul_2*
T0
¿
?gradients/batch_normalization_3/cond/batchnorm/mul_2_grad/Sum_1Sum?gradients/batch_normalization_3/cond/batchnorm/mul_2_grad/mul_1Qgradients/batch_normalization_3/cond/batchnorm/mul_2_grad/BroadcastGradientArgs:1*=
_class3
1/loc:@batch_normalization_3/cond/batchnorm/mul_2*
T0*
	keep_dims( *

Tidx0
¨
Cgradients/batch_normalization_3/cond/batchnorm/mul_2_grad/Reshape_1Reshape?gradients/batch_normalization_3/cond/batchnorm/mul_2_grad/Sum_1Agradients/batch_normalization_3/cond/batchnorm/mul_2_grad/Shape_1*=
_class3
1/loc:@batch_normalization_3/cond/batchnorm/mul_2*
T0*
Tshape0

:gradients/batch_normalization_3/batchnorm/mul_1_grad/ShapeShapeh2/Relu*
out_type0*
T0*8
_class.
,*loc:@batch_normalization_3/batchnorm/mul_1
¤
<gradients/batch_normalization_3/batchnorm/mul_1_grad/Shape_1Const*
dtype0*8
_class.
,*loc:@batch_normalization_3/batchnorm/mul_1*
valueB:d
 
Jgradients/batch_normalization_3/batchnorm/mul_1_grad/BroadcastGradientArgsBroadcastGradientArgs:gradients/batch_normalization_3/batchnorm/mul_1_grad/Shape<gradients/batch_normalization_3/batchnorm/mul_1_grad/Shape_1*8
_class.
,*loc:@batch_normalization_3/batchnorm/mul_1*
T0
å
8gradients/batch_normalization_3/batchnorm/mul_1_grad/mulMul<gradients/batch_normalization_3/batchnorm/add_1_grad/Reshape#batch_normalization_3/batchnorm/mul*8
_class.
,*loc:@batch_normalization_3/batchnorm/mul_1*
T0
¥
8gradients/batch_normalization_3/batchnorm/mul_1_grad/SumSum8gradients/batch_normalization_3/batchnorm/mul_1_grad/mulJgradients/batch_normalization_3/batchnorm/mul_1_grad/BroadcastGradientArgs*8
_class.
,*loc:@batch_normalization_3/batchnorm/mul_1*
T0*
	keep_dims( *

Tidx0

<gradients/batch_normalization_3/batchnorm/mul_1_grad/ReshapeReshape8gradients/batch_normalization_3/batchnorm/mul_1_grad/Sum:gradients/batch_normalization_3/batchnorm/mul_1_grad/Shape*8
_class.
,*loc:@batch_normalization_3/batchnorm/mul_1*
T0*
Tshape0
Ë
:gradients/batch_normalization_3/batchnorm/mul_1_grad/mul_1Mulh2/Relu<gradients/batch_normalization_3/batchnorm/add_1_grad/Reshape*8
_class.
,*loc:@batch_normalization_3/batchnorm/mul_1*
T0
«
:gradients/batch_normalization_3/batchnorm/mul_1_grad/Sum_1Sum:gradients/batch_normalization_3/batchnorm/mul_1_grad/mul_1Lgradients/batch_normalization_3/batchnorm/mul_1_grad/BroadcastGradientArgs:1*8
_class.
,*loc:@batch_normalization_3/batchnorm/mul_1*
T0*
	keep_dims( *

Tidx0

>gradients/batch_normalization_3/batchnorm/mul_1_grad/Reshape_1Reshape:gradients/batch_normalization_3/batchnorm/mul_1_grad/Sum_1<gradients/batch_normalization_3/batchnorm/mul_1_grad/Shape_1*8
_class.
,*loc:@batch_normalization_3/batchnorm/mul_1*
T0*
Tshape0

8gradients/batch_normalization_3/batchnorm/sub_grad/ShapeConst*
dtype0*6
_class,
*(loc:@batch_normalization_3/batchnorm/sub*
valueB:d
 
:gradients/batch_normalization_3/batchnorm/sub_grad/Shape_1Const*
dtype0*6
_class,
*(loc:@batch_normalization_3/batchnorm/sub*
valueB:d

Hgradients/batch_normalization_3/batchnorm/sub_grad/BroadcastGradientArgsBroadcastGradientArgs8gradients/batch_normalization_3/batchnorm/sub_grad/Shape:gradients/batch_normalization_3/batchnorm/sub_grad/Shape_1*6
_class,
*(loc:@batch_normalization_3/batchnorm/sub*
T0
¥
6gradients/batch_normalization_3/batchnorm/sub_grad/SumSum>gradients/batch_normalization_3/batchnorm/add_1_grad/Reshape_1Hgradients/batch_normalization_3/batchnorm/sub_grad/BroadcastGradientArgs*6
_class,
*(loc:@batch_normalization_3/batchnorm/sub*
T0*
	keep_dims( *

Tidx0

:gradients/batch_normalization_3/batchnorm/sub_grad/ReshapeReshape6gradients/batch_normalization_3/batchnorm/sub_grad/Sum8gradients/batch_normalization_3/batchnorm/sub_grad/Shape*6
_class,
*(loc:@batch_normalization_3/batchnorm/sub*
T0*
Tshape0
©
8gradients/batch_normalization_3/batchnorm/sub_grad/Sum_1Sum>gradients/batch_normalization_3/batchnorm/add_1_grad/Reshape_1Jgradients/batch_normalization_3/batchnorm/sub_grad/BroadcastGradientArgs:1*6
_class,
*(loc:@batch_normalization_3/batchnorm/sub*
T0*
	keep_dims( *

Tidx0
¸
6gradients/batch_normalization_3/batchnorm/sub_grad/NegNeg8gradients/batch_normalization_3/batchnorm/sub_grad/Sum_1*6
_class,
*(loc:@batch_normalization_3/batchnorm/sub*
T0

<gradients/batch_normalization_3/batchnorm/sub_grad/Reshape_1Reshape6gradients/batch_normalization_3/batchnorm/sub_grad/Neg:gradients/batch_normalization_3/batchnorm/sub_grad/Shape_1*6
_class,
*(loc:@batch_normalization_3/batchnorm/sub*
T0*
Tshape0
ó
gradients/AddN_1AddNCgradients/batch_normalization_3/cond/batchnorm/mul_1_grad/Reshape_1Cgradients/batch_normalization_3/cond/batchnorm/mul_2_grad/Reshape_1*=
_class3
1/loc:@batch_normalization_3/cond/batchnorm/mul_1*
T0*
N
¨
=gradients/batch_normalization_3/cond/batchnorm/mul_grad/ShapeConst*
dtype0*;
_class1
/-loc:@batch_normalization_3/cond/batchnorm/mul*
valueB:d
ª
?gradients/batch_normalization_3/cond/batchnorm/mul_grad/Shape_1Const*
dtype0*;
_class1
/-loc:@batch_normalization_3/cond/batchnorm/mul*
valueB:d
¬
Mgradients/batch_normalization_3/cond/batchnorm/mul_grad/BroadcastGradientArgsBroadcastGradientArgs=gradients/batch_normalization_3/cond/batchnorm/mul_grad/Shape?gradients/batch_normalization_3/cond/batchnorm/mul_grad/Shape_1*;
_class1
/-loc:@batch_normalization_3/cond/batchnorm/mul*
T0
Ë
;gradients/batch_normalization_3/cond/batchnorm/mul_grad/mulMulgradients/AddN_1/batch_normalization_3/cond/batchnorm/mul/Switch*;
_class1
/-loc:@batch_normalization_3/cond/batchnorm/mul*
T0
±
;gradients/batch_normalization_3/cond/batchnorm/mul_grad/SumSum;gradients/batch_normalization_3/cond/batchnorm/mul_grad/mulMgradients/batch_normalization_3/cond/batchnorm/mul_grad/BroadcastGradientArgs*;
_class1
/-loc:@batch_normalization_3/cond/batchnorm/mul*
T0*
	keep_dims( *

Tidx0

?gradients/batch_normalization_3/cond/batchnorm/mul_grad/ReshapeReshape;gradients/batch_normalization_3/cond/batchnorm/mul_grad/Sum=gradients/batch_normalization_3/cond/batchnorm/mul_grad/Shape*;
_class1
/-loc:@batch_normalization_3/cond/batchnorm/mul*
T0*
Tshape0
È
=gradients/batch_normalization_3/cond/batchnorm/mul_grad/mul_1Mul*batch_normalization_3/cond/batchnorm/Rsqrtgradients/AddN_1*;
_class1
/-loc:@batch_normalization_3/cond/batchnorm/mul*
T0
·
=gradients/batch_normalization_3/cond/batchnorm/mul_grad/Sum_1Sum=gradients/batch_normalization_3/cond/batchnorm/mul_grad/mul_1Ogradients/batch_normalization_3/cond/batchnorm/mul_grad/BroadcastGradientArgs:1*;
_class1
/-loc:@batch_normalization_3/cond/batchnorm/mul*
T0*
	keep_dims( *

Tidx0
 
Agradients/batch_normalization_3/cond/batchnorm/mul_grad/Reshape_1Reshape=gradients/batch_normalization_3/cond/batchnorm/mul_grad/Sum_1?gradients/batch_normalization_3/cond/batchnorm/mul_grad/Shape_1*;
_class1
/-loc:@batch_normalization_3/cond/batchnorm/mul*
T0*
Tshape0
ß
gradients/AddN_2AddNHgradients/batch_normalization_3/cond/batchnorm/sub/Switch_grad/cond_grad:gradients/batch_normalization_3/batchnorm/sub_grad/Reshape*-
_class#
!loc:@batch_normalization_3/beta*
T0*
N
¢
:gradients/batch_normalization_3/batchnorm/mul_2_grad/ShapeConst*
dtype0*8
_class.
,*loc:@batch_normalization_3/batchnorm/mul_2*
valueB:d
¤
<gradients/batch_normalization_3/batchnorm/mul_2_grad/Shape_1Const*
dtype0*8
_class.
,*loc:@batch_normalization_3/batchnorm/mul_2*
valueB:d
 
Jgradients/batch_normalization_3/batchnorm/mul_2_grad/BroadcastGradientArgsBroadcastGradientArgs:gradients/batch_normalization_3/batchnorm/mul_2_grad/Shape<gradients/batch_normalization_3/batchnorm/mul_2_grad/Shape_1*8
_class.
,*loc:@batch_normalization_3/batchnorm/mul_2*
T0
å
8gradients/batch_normalization_3/batchnorm/mul_2_grad/mulMul<gradients/batch_normalization_3/batchnorm/sub_grad/Reshape_1#batch_normalization_3/batchnorm/mul*8
_class.
,*loc:@batch_normalization_3/batchnorm/mul_2*
T0
¥
8gradients/batch_normalization_3/batchnorm/mul_2_grad/SumSum8gradients/batch_normalization_3/batchnorm/mul_2_grad/mulJgradients/batch_normalization_3/batchnorm/mul_2_grad/BroadcastGradientArgs*8
_class.
,*loc:@batch_normalization_3/batchnorm/mul_2*
T0*
	keep_dims( *

Tidx0

<gradients/batch_normalization_3/batchnorm/mul_2_grad/ReshapeReshape8gradients/batch_normalization_3/batchnorm/mul_2_grad/Sum:gradients/batch_normalization_3/batchnorm/mul_2_grad/Shape*8
_class.
,*loc:@batch_normalization_3/batchnorm/mul_2*
T0*
Tshape0
ð
:gradients/batch_normalization_3/batchnorm/mul_2_grad/mul_1Mul,batch_normalization_3/moments/normalize/mean<gradients/batch_normalization_3/batchnorm/sub_grad/Reshape_1*8
_class.
,*loc:@batch_normalization_3/batchnorm/mul_2*
T0
«
:gradients/batch_normalization_3/batchnorm/mul_2_grad/Sum_1Sum:gradients/batch_normalization_3/batchnorm/mul_2_grad/mul_1Lgradients/batch_normalization_3/batchnorm/mul_2_grad/BroadcastGradientArgs:1*8
_class.
,*loc:@batch_normalization_3/batchnorm/mul_2*
T0*
	keep_dims( *

Tidx0

>gradients/batch_normalization_3/batchnorm/mul_2_grad/Reshape_1Reshape:gradients/batch_normalization_3/batchnorm/mul_2_grad/Sum_1<gradients/batch_normalization_3/batchnorm/mul_2_grad/Shape_1*8
_class.
,*loc:@batch_normalization_3/batchnorm/mul_2*
T0*
Tshape0

gradients/Switch_3Switch batch_normalization_3/gamma/read"batch_normalization_3/cond/pred_id*.
_class$
" loc:@batch_normalization_3/gamma*
T0
y
gradients/Shape_4Shapegradients/Switch_3:1*
out_type0*
T0*.
_class$
" loc:@batch_normalization_3/gamma
t
gradients/zeros_3/ConstConst*
dtype0*.
_class$
" loc:@batch_normalization_3/gamma*
valueB
 *    
~
gradients/zeros_3Fillgradients/Shape_4gradients/zeros_3/Const*.
_class$
" loc:@batch_normalization_3/gamma*
T0
é
Hgradients/batch_normalization_3/cond/batchnorm/mul/Switch_grad/cond_gradMergeAgradients/batch_normalization_3/cond/batchnorm/mul_grad/Reshape_1gradients/zeros_3*.
_class$
" loc:@batch_normalization_3/gamma*
T0*
N
ä
gradients/AddN_3AddN>gradients/batch_normalization_3/batchnorm/mul_1_grad/Reshape_1>gradients/batch_normalization_3/batchnorm/mul_2_grad/Reshape_1*8
_class.
,*loc:@batch_normalization_3/batchnorm/mul_1*
T0*
N

8gradients/batch_normalization_3/batchnorm/mul_grad/ShapeConst*
dtype0*6
_class,
*(loc:@batch_normalization_3/batchnorm/mul*
valueB:d
 
:gradients/batch_normalization_3/batchnorm/mul_grad/Shape_1Const*
dtype0*6
_class,
*(loc:@batch_normalization_3/batchnorm/mul*
valueB:d

Hgradients/batch_normalization_3/batchnorm/mul_grad/BroadcastGradientArgsBroadcastGradientArgs8gradients/batch_normalization_3/batchnorm/mul_grad/Shape:gradients/batch_normalization_3/batchnorm/mul_grad/Shape_1*6
_class,
*(loc:@batch_normalization_3/batchnorm/mul*
T0
²
6gradients/batch_normalization_3/batchnorm/mul_grad/mulMulgradients/AddN_3 batch_normalization_3/gamma/read*6
_class,
*(loc:@batch_normalization_3/batchnorm/mul*
T0

6gradients/batch_normalization_3/batchnorm/mul_grad/SumSum6gradients/batch_normalization_3/batchnorm/mul_grad/mulHgradients/batch_normalization_3/batchnorm/mul_grad/BroadcastGradientArgs*6
_class,
*(loc:@batch_normalization_3/batchnorm/mul*
T0*
	keep_dims( *

Tidx0

:gradients/batch_normalization_3/batchnorm/mul_grad/ReshapeReshape6gradients/batch_normalization_3/batchnorm/mul_grad/Sum8gradients/batch_normalization_3/batchnorm/mul_grad/Shape*6
_class,
*(loc:@batch_normalization_3/batchnorm/mul*
T0*
Tshape0
¹
8gradients/batch_normalization_3/batchnorm/mul_grad/mul_1Mul%batch_normalization_3/batchnorm/Rsqrtgradients/AddN_3*6
_class,
*(loc:@batch_normalization_3/batchnorm/mul*
T0
£
8gradients/batch_normalization_3/batchnorm/mul_grad/Sum_1Sum8gradients/batch_normalization_3/batchnorm/mul_grad/mul_1Jgradients/batch_normalization_3/batchnorm/mul_grad/BroadcastGradientArgs:1*6
_class,
*(loc:@batch_normalization_3/batchnorm/mul*
T0*
	keep_dims( *

Tidx0

<gradients/batch_normalization_3/batchnorm/mul_grad/Reshape_1Reshape8gradients/batch_normalization_3/batchnorm/mul_grad/Sum_1:gradients/batch_normalization_3/batchnorm/mul_grad/Shape_1*6
_class,
*(loc:@batch_normalization_3/batchnorm/mul*
T0*
Tshape0
ñ
>gradients/batch_normalization_3/batchnorm/Rsqrt_grad/RsqrtGrad	RsqrtGrad%batch_normalization_3/batchnorm/Rsqrt:gradients/batch_normalization_3/batchnorm/mul_grad/Reshape*8
_class.
,*loc:@batch_normalization_3/batchnorm/Rsqrt*
T0
â
gradients/AddN_4AddNHgradients/batch_normalization_3/cond/batchnorm/mul/Switch_grad/cond_grad<gradients/batch_normalization_3/batchnorm/mul_grad/Reshape_1*.
_class$
" loc:@batch_normalization_3/gamma*
T0*
N

8gradients/batch_normalization_3/batchnorm/add_grad/ShapeConst*
dtype0*6
_class,
*(loc:@batch_normalization_3/batchnorm/add*
valueB:d

:gradients/batch_normalization_3/batchnorm/add_grad/Shape_1Const*
dtype0*6
_class,
*(loc:@batch_normalization_3/batchnorm/add*
valueB 

Hgradients/batch_normalization_3/batchnorm/add_grad/BroadcastGradientArgsBroadcastGradientArgs8gradients/batch_normalization_3/batchnorm/add_grad/Shape:gradients/batch_normalization_3/batchnorm/add_grad/Shape_1*6
_class,
*(loc:@batch_normalization_3/batchnorm/add*
T0
¥
6gradients/batch_normalization_3/batchnorm/add_grad/SumSum>gradients/batch_normalization_3/batchnorm/Rsqrt_grad/RsqrtGradHgradients/batch_normalization_3/batchnorm/add_grad/BroadcastGradientArgs*6
_class,
*(loc:@batch_normalization_3/batchnorm/add*
T0*
	keep_dims( *

Tidx0

:gradients/batch_normalization_3/batchnorm/add_grad/ReshapeReshape6gradients/batch_normalization_3/batchnorm/add_grad/Sum8gradients/batch_normalization_3/batchnorm/add_grad/Shape*6
_class,
*(loc:@batch_normalization_3/batchnorm/add*
T0*
Tshape0
©
8gradients/batch_normalization_3/batchnorm/add_grad/Sum_1Sum>gradients/batch_normalization_3/batchnorm/Rsqrt_grad/RsqrtGradJgradients/batch_normalization_3/batchnorm/add_grad/BroadcastGradientArgs:1*6
_class,
*(loc:@batch_normalization_3/batchnorm/add*
T0*
	keep_dims( *

Tidx0

<gradients/batch_normalization_3/batchnorm/add_grad/Reshape_1Reshape8gradients/batch_normalization_3/batchnorm/add_grad/Sum_1:gradients/batch_normalization_3/batchnorm/add_grad/Shape_1*6
_class,
*(loc:@batch_normalization_3/batchnorm/add*
T0*
Tshape0
¸
Egradients/batch_normalization_3/moments/normalize/variance_grad/ShapeConst*
dtype0*C
_class9
75loc:@batch_normalization_3/moments/normalize/variance*
valueB:d
º
Ggradients/batch_normalization_3/moments/normalize/variance_grad/Shape_1Const*
dtype0*C
_class9
75loc:@batch_normalization_3/moments/normalize/variance*
valueB:d
Ì
Ugradients/batch_normalization_3/moments/normalize/variance_grad/BroadcastGradientArgsBroadcastGradientArgsEgradients/batch_normalization_3/moments/normalize/variance_grad/ShapeGgradients/batch_normalization_3/moments/normalize/variance_grad/Shape_1*C
_class9
75loc:@batch_normalization_3/moments/normalize/variance*
T0
È
Cgradients/batch_normalization_3/moments/normalize/variance_grad/SumSum:gradients/batch_normalization_3/batchnorm/add_grad/ReshapeUgradients/batch_normalization_3/moments/normalize/variance_grad/BroadcastGradientArgs*C
_class9
75loc:@batch_normalization_3/moments/normalize/variance*
T0*
	keep_dims( *

Tidx0
º
Ggradients/batch_normalization_3/moments/normalize/variance_grad/ReshapeReshapeCgradients/batch_normalization_3/moments/normalize/variance_grad/SumEgradients/batch_normalization_3/moments/normalize/variance_grad/Shape*C
_class9
75loc:@batch_normalization_3/moments/normalize/variance*
T0*
Tshape0
Ì
Egradients/batch_normalization_3/moments/normalize/variance_grad/Sum_1Sum:gradients/batch_normalization_3/batchnorm/add_grad/ReshapeWgradients/batch_normalization_3/moments/normalize/variance_grad/BroadcastGradientArgs:1*C
_class9
75loc:@batch_normalization_3/moments/normalize/variance*
T0*
	keep_dims( *

Tidx0
ß
Cgradients/batch_normalization_3/moments/normalize/variance_grad/NegNegEgradients/batch_normalization_3/moments/normalize/variance_grad/Sum_1*C
_class9
75loc:@batch_normalization_3/moments/normalize/variance*
T0
¾
Igradients/batch_normalization_3/moments/normalize/variance_grad/Reshape_1ReshapeCgradients/batch_normalization_3/moments/normalize/variance_grad/NegGgradients/batch_normalization_3/moments/normalize/variance_grad/Shape_1*C
_class9
75loc:@batch_normalization_3/moments/normalize/variance*
T0*
Tshape0
®
@gradients/batch_normalization_3/moments/normalize/Mul_grad/ShapeConst*
dtype0*>
_class4
20loc:@batch_normalization_3/moments/normalize/Mul*
valueB:d
«
Bgradients/batch_normalization_3/moments/normalize/Mul_grad/Shape_1Const*
dtype0*>
_class4
20loc:@batch_normalization_3/moments/normalize/Mul*
valueB 
¸
Pgradients/batch_normalization_3/moments/normalize/Mul_grad/BroadcastGradientArgsBroadcastGradientArgs@gradients/batch_normalization_3/moments/normalize/Mul_grad/ShapeBgradients/batch_normalization_3/moments/normalize/Mul_grad/Shape_1*>
_class4
20loc:@batch_normalization_3/moments/normalize/Mul*
T0

>gradients/batch_normalization_3/moments/normalize/Mul_grad/mulMulGgradients/batch_normalization_3/moments/normalize/variance_grad/Reshape/batch_normalization_3/moments/normalize/divisor*>
_class4
20loc:@batch_normalization_3/moments/normalize/Mul*
T0
½
>gradients/batch_normalization_3/moments/normalize/Mul_grad/SumSum>gradients/batch_normalization_3/moments/normalize/Mul_grad/mulPgradients/batch_normalization_3/moments/normalize/Mul_grad/BroadcastGradientArgs*>
_class4
20loc:@batch_normalization_3/moments/normalize/Mul*
T0*
	keep_dims( *

Tidx0
¦
Bgradients/batch_normalization_3/moments/normalize/Mul_grad/ReshapeReshape>gradients/batch_normalization_3/moments/normalize/Mul_grad/Sum@gradients/batch_normalization_3/moments/normalize/Mul_grad/Shape*>
_class4
20loc:@batch_normalization_3/moments/normalize/Mul*
T0*
Tshape0

@gradients/batch_normalization_3/moments/normalize/Mul_grad/mul_1Mul:batch_normalization_3/moments/sufficient_statistics/var_ssGgradients/batch_normalization_3/moments/normalize/variance_grad/Reshape*>
_class4
20loc:@batch_normalization_3/moments/normalize/Mul*
T0
Ã
@gradients/batch_normalization_3/moments/normalize/Mul_grad/Sum_1Sum@gradients/batch_normalization_3/moments/normalize/Mul_grad/mul_1Rgradients/batch_normalization_3/moments/normalize/Mul_grad/BroadcastGradientArgs:1*>
_class4
20loc:@batch_normalization_3/moments/normalize/Mul*
T0*
	keep_dims( *

Tidx0
¬
Dgradients/batch_normalization_3/moments/normalize/Mul_grad/Reshape_1Reshape@gradients/batch_normalization_3/moments/normalize/Mul_grad/Sum_1Bgradients/batch_normalization_3/moments/normalize/Mul_grad/Shape_1*>
_class4
20loc:@batch_normalization_3/moments/normalize/Mul*
T0*
Tshape0
ÿ
Cgradients/batch_normalization_3/moments/normalize/Square_grad/mul/xConstJ^gradients/batch_normalization_3/moments/normalize/variance_grad/Reshape_1*
dtype0*A
_class7
53loc:@batch_normalization_3/moments/normalize/Square*
valueB
 *   @

Agradients/batch_normalization_3/moments/normalize/Square_grad/mulMulCgradients/batch_normalization_3/moments/normalize/Square_grad/mul/x,batch_normalization_3/moments/normalize/mean*A
_class7
53loc:@batch_normalization_3/moments/normalize/Square*
T0
¤
Cgradients/batch_normalization_3/moments/normalize/Square_grad/mul_1MulIgradients/batch_normalization_3/moments/normalize/variance_grad/Reshape_1Agradients/batch_normalization_3/moments/normalize/Square_grad/mul*A
_class7
53loc:@batch_normalization_3/moments/normalize/Square*
T0
ü
Ogradients/batch_normalization_3/moments/sufficient_statistics/var_ss_grad/ShapeShape:batch_normalization_3/moments/sufficient_statistics/Square*
out_type0*
T0*M
_classC
A?loc:@batch_normalization_3/moments/sufficient_statistics/var_ss
Ç
Ngradients/batch_normalization_3/moments/sufficient_statistics/var_ss_grad/SizeConst*
dtype0*M
_classC
A?loc:@batch_normalization_3/moments/sufficient_statistics/var_ss*
value	B :
Ê
Mgradients/batch_normalization_3/moments/sufficient_statistics/var_ss_grad/addAddLbatch_normalization_3/moments/sufficient_statistics/var_ss/reduction_indicesNgradients/batch_normalization_3/moments/sufficient_statistics/var_ss_grad/Size*M
_classC
A?loc:@batch_normalization_3/moments/sufficient_statistics/var_ss*
T0
Ë
Mgradients/batch_normalization_3/moments/sufficient_statistics/var_ss_grad/modModMgradients/batch_normalization_3/moments/sufficient_statistics/var_ss_grad/addNgradients/batch_normalization_3/moments/sufficient_statistics/var_ss_grad/Size*M
_classC
A?loc:@batch_normalization_3/moments/sufficient_statistics/var_ss*
T0
Î
Qgradients/batch_normalization_3/moments/sufficient_statistics/var_ss_grad/Shape_1Const*
dtype0*M
_classC
A?loc:@batch_normalization_3/moments/sufficient_statistics/var_ss*
valueB:
Î
Ugradients/batch_normalization_3/moments/sufficient_statistics/var_ss_grad/range/startConst*
dtype0*M
_classC
A?loc:@batch_normalization_3/moments/sufficient_statistics/var_ss*
value	B : 
Î
Ugradients/batch_normalization_3/moments/sufficient_statistics/var_ss_grad/range/deltaConst*
dtype0*M
_classC
A?loc:@batch_normalization_3/moments/sufficient_statistics/var_ss*
value	B :
±
Ogradients/batch_normalization_3/moments/sufficient_statistics/var_ss_grad/rangeRangeUgradients/batch_normalization_3/moments/sufficient_statistics/var_ss_grad/range/startNgradients/batch_normalization_3/moments/sufficient_statistics/var_ss_grad/SizeUgradients/batch_normalization_3/moments/sufficient_statistics/var_ss_grad/range/delta*M
_classC
A?loc:@batch_normalization_3/moments/sufficient_statistics/var_ss*

Tidx0
Í
Tgradients/batch_normalization_3/moments/sufficient_statistics/var_ss_grad/Fill/valueConst*
dtype0*M
_classC
A?loc:@batch_normalization_3/moments/sufficient_statistics/var_ss*
value	B :
×
Ngradients/batch_normalization_3/moments/sufficient_statistics/var_ss_grad/FillFillQgradients/batch_normalization_3/moments/sufficient_statistics/var_ss_grad/Shape_1Tgradients/batch_normalization_3/moments/sufficient_statistics/var_ss_grad/Fill/value*M
_classC
A?loc:@batch_normalization_3/moments/sufficient_statistics/var_ss*
T0

Wgradients/batch_normalization_3/moments/sufficient_statistics/var_ss_grad/DynamicStitchDynamicStitchOgradients/batch_normalization_3/moments/sufficient_statistics/var_ss_grad/rangeMgradients/batch_normalization_3/moments/sufficient_statistics/var_ss_grad/modOgradients/batch_normalization_3/moments/sufficient_statistics/var_ss_grad/ShapeNgradients/batch_normalization_3/moments/sufficient_statistics/var_ss_grad/Fill*M
_classC
A?loc:@batch_normalization_3/moments/sufficient_statistics/var_ss*
T0*
N
Ì
Sgradients/batch_normalization_3/moments/sufficient_statistics/var_ss_grad/Maximum/yConst*
dtype0*M
_classC
A?loc:@batch_normalization_3/moments/sufficient_statistics/var_ss*
value	B :
â
Qgradients/batch_normalization_3/moments/sufficient_statistics/var_ss_grad/MaximumMaximumWgradients/batch_normalization_3/moments/sufficient_statistics/var_ss_grad/DynamicStitchSgradients/batch_normalization_3/moments/sufficient_statistics/var_ss_grad/Maximum/y*M
_classC
A?loc:@batch_normalization_3/moments/sufficient_statistics/var_ss*
T0
Õ
Rgradients/batch_normalization_3/moments/sufficient_statistics/var_ss_grad/floordivDivOgradients/batch_normalization_3/moments/sufficient_statistics/var_ss_grad/ShapeQgradients/batch_normalization_3/moments/sufficient_statistics/var_ss_grad/Maximum*M
_classC
A?loc:@batch_normalization_3/moments/sufficient_statistics/var_ss*
T0
ß
Qgradients/batch_normalization_3/moments/sufficient_statistics/var_ss_grad/ReshapeReshapeBgradients/batch_normalization_3/moments/normalize/Mul_grad/ReshapeWgradients/batch_normalization_3/moments/sufficient_statistics/var_ss_grad/DynamicStitch*M
_classC
A?loc:@batch_normalization_3/moments/sufficient_statistics/var_ss*
T0*
Tshape0
ç
Ngradients/batch_normalization_3/moments/sufficient_statistics/var_ss_grad/TileTileQgradients/batch_normalization_3/moments/sufficient_statistics/var_ss_grad/ReshapeRgradients/batch_normalization_3/moments/sufficient_statistics/var_ss_grad/floordiv*

Tmultiples0*M
_classC
A?loc:@batch_normalization_3/moments/sufficient_statistics/var_ss*
T0
ç
gradients/AddN_5AddN<gradients/batch_normalization_3/batchnorm/mul_2_grad/ReshapeCgradients/batch_normalization_3/moments/normalize/Square_grad/mul_1*8
_class.
,*loc:@batch_normalization_3/batchnorm/mul_2*
T0*
N
°
Agradients/batch_normalization_3/moments/normalize/mean_grad/ShapeConst*
dtype0*?
_class5
31loc:@batch_normalization_3/moments/normalize/mean*
valueB:d
­
Cgradients/batch_normalization_3/moments/normalize/mean_grad/Shape_1Const*
dtype0*?
_class5
31loc:@batch_normalization_3/moments/normalize/mean*
valueB 
¼
Qgradients/batch_normalization_3/moments/normalize/mean_grad/BroadcastGradientArgsBroadcastGradientArgsAgradients/batch_normalization_3/moments/normalize/mean_grad/ShapeCgradients/batch_normalization_3/moments/normalize/mean_grad/Shape_1*?
_class5
31loc:@batch_normalization_3/moments/normalize/mean*
T0
Ó
?gradients/batch_normalization_3/moments/normalize/mean_grad/mulMulgradients/AddN_5/batch_normalization_3/moments/normalize/divisor*?
_class5
31loc:@batch_normalization_3/moments/normalize/mean*
T0
Á
?gradients/batch_normalization_3/moments/normalize/mean_grad/SumSum?gradients/batch_normalization_3/moments/normalize/mean_grad/mulQgradients/batch_normalization_3/moments/normalize/mean_grad/BroadcastGradientArgs*?
_class5
31loc:@batch_normalization_3/moments/normalize/mean*
T0*
	keep_dims( *

Tidx0
ª
Cgradients/batch_normalization_3/moments/normalize/mean_grad/ReshapeReshape?gradients/batch_normalization_3/moments/normalize/mean_grad/SumAgradients/batch_normalization_3/moments/normalize/mean_grad/Shape*?
_class5
31loc:@batch_normalization_3/moments/normalize/mean*
T0*
Tshape0
á
Agradients/batch_normalization_3/moments/normalize/mean_grad/mul_1Mul;batch_normalization_3/moments/sufficient_statistics/mean_ssgradients/AddN_5*?
_class5
31loc:@batch_normalization_3/moments/normalize/mean*
T0
Ç
Agradients/batch_normalization_3/moments/normalize/mean_grad/Sum_1SumAgradients/batch_normalization_3/moments/normalize/mean_grad/mul_1Sgradients/batch_normalization_3/moments/normalize/mean_grad/BroadcastGradientArgs:1*?
_class5
31loc:@batch_normalization_3/moments/normalize/mean*
T0*
	keep_dims( *

Tidx0
°
Egradients/batch_normalization_3/moments/normalize/mean_grad/Reshape_1ReshapeAgradients/batch_normalization_3/moments/normalize/mean_grad/Sum_1Cgradients/batch_normalization_3/moments/normalize/mean_grad/Shape_1*?
_class5
31loc:@batch_normalization_3/moments/normalize/mean*
T0*
Tshape0

Ogradients/batch_normalization_3/moments/sufficient_statistics/Square_grad/mul/xConstO^gradients/batch_normalization_3/moments/sufficient_statistics/var_ss_grad/Tile*
dtype0*M
_classC
A?loc:@batch_normalization_3/moments/sufficient_statistics/Square*
valueB
 *   @

Mgradients/batch_normalization_3/moments/sufficient_statistics/Square_grad/mulMulOgradients/batch_normalization_3/moments/sufficient_statistics/Square_grad/mul/xh2/Relu*M
_classC
A?loc:@batch_normalization_3/moments/sufficient_statistics/Square*
T0
Í
Ogradients/batch_normalization_3/moments/sufficient_statistics/Square_grad/mul_1MulNgradients/batch_normalization_3/moments/sufficient_statistics/var_ss_grad/TileMgradients/batch_normalization_3/moments/sufficient_statistics/Square_grad/mul*M
_classC
A?loc:@batch_normalization_3/moments/sufficient_statistics/Square*
T0
Ë
Pgradients/batch_normalization_3/moments/sufficient_statistics/mean_ss_grad/ShapeShapeh2/Relu*
out_type0*
T0*N
_classD
B@loc:@batch_normalization_3/moments/sufficient_statistics/mean_ss
É
Ogradients/batch_normalization_3/moments/sufficient_statistics/mean_ss_grad/SizeConst*
dtype0*N
_classD
B@loc:@batch_normalization_3/moments/sufficient_statistics/mean_ss*
value	B :
Î
Ngradients/batch_normalization_3/moments/sufficient_statistics/mean_ss_grad/addAddMbatch_normalization_3/moments/sufficient_statistics/mean_ss/reduction_indicesOgradients/batch_normalization_3/moments/sufficient_statistics/mean_ss_grad/Size*N
_classD
B@loc:@batch_normalization_3/moments/sufficient_statistics/mean_ss*
T0
Ï
Ngradients/batch_normalization_3/moments/sufficient_statistics/mean_ss_grad/modModNgradients/batch_normalization_3/moments/sufficient_statistics/mean_ss_grad/addOgradients/batch_normalization_3/moments/sufficient_statistics/mean_ss_grad/Size*N
_classD
B@loc:@batch_normalization_3/moments/sufficient_statistics/mean_ss*
T0
Ð
Rgradients/batch_normalization_3/moments/sufficient_statistics/mean_ss_grad/Shape_1Const*
dtype0*N
_classD
B@loc:@batch_normalization_3/moments/sufficient_statistics/mean_ss*
valueB:
Ð
Vgradients/batch_normalization_3/moments/sufficient_statistics/mean_ss_grad/range/startConst*
dtype0*N
_classD
B@loc:@batch_normalization_3/moments/sufficient_statistics/mean_ss*
value	B : 
Ð
Vgradients/batch_normalization_3/moments/sufficient_statistics/mean_ss_grad/range/deltaConst*
dtype0*N
_classD
B@loc:@batch_normalization_3/moments/sufficient_statistics/mean_ss*
value	B :
¶
Pgradients/batch_normalization_3/moments/sufficient_statistics/mean_ss_grad/rangeRangeVgradients/batch_normalization_3/moments/sufficient_statistics/mean_ss_grad/range/startOgradients/batch_normalization_3/moments/sufficient_statistics/mean_ss_grad/SizeVgradients/batch_normalization_3/moments/sufficient_statistics/mean_ss_grad/range/delta*N
_classD
B@loc:@batch_normalization_3/moments/sufficient_statistics/mean_ss*

Tidx0
Ï
Ugradients/batch_normalization_3/moments/sufficient_statistics/mean_ss_grad/Fill/valueConst*
dtype0*N
_classD
B@loc:@batch_normalization_3/moments/sufficient_statistics/mean_ss*
value	B :
Û
Ogradients/batch_normalization_3/moments/sufficient_statistics/mean_ss_grad/FillFillRgradients/batch_normalization_3/moments/sufficient_statistics/mean_ss_grad/Shape_1Ugradients/batch_normalization_3/moments/sufficient_statistics/mean_ss_grad/Fill/value*N
_classD
B@loc:@batch_normalization_3/moments/sufficient_statistics/mean_ss*
T0

Xgradients/batch_normalization_3/moments/sufficient_statistics/mean_ss_grad/DynamicStitchDynamicStitchPgradients/batch_normalization_3/moments/sufficient_statistics/mean_ss_grad/rangeNgradients/batch_normalization_3/moments/sufficient_statistics/mean_ss_grad/modPgradients/batch_normalization_3/moments/sufficient_statistics/mean_ss_grad/ShapeOgradients/batch_normalization_3/moments/sufficient_statistics/mean_ss_grad/Fill*N
_classD
B@loc:@batch_normalization_3/moments/sufficient_statistics/mean_ss*
T0*
N
Î
Tgradients/batch_normalization_3/moments/sufficient_statistics/mean_ss_grad/Maximum/yConst*
dtype0*N
_classD
B@loc:@batch_normalization_3/moments/sufficient_statistics/mean_ss*
value	B :
æ
Rgradients/batch_normalization_3/moments/sufficient_statistics/mean_ss_grad/MaximumMaximumXgradients/batch_normalization_3/moments/sufficient_statistics/mean_ss_grad/DynamicStitchTgradients/batch_normalization_3/moments/sufficient_statistics/mean_ss_grad/Maximum/y*N
_classD
B@loc:@batch_normalization_3/moments/sufficient_statistics/mean_ss*
T0
Ù
Sgradients/batch_normalization_3/moments/sufficient_statistics/mean_ss_grad/floordivDivPgradients/batch_normalization_3/moments/sufficient_statistics/mean_ss_grad/ShapeRgradients/batch_normalization_3/moments/sufficient_statistics/mean_ss_grad/Maximum*N
_classD
B@loc:@batch_normalization_3/moments/sufficient_statistics/mean_ss*
T0
ã
Rgradients/batch_normalization_3/moments/sufficient_statistics/mean_ss_grad/ReshapeReshapeCgradients/batch_normalization_3/moments/normalize/mean_grad/ReshapeXgradients/batch_normalization_3/moments/sufficient_statistics/mean_ss_grad/DynamicStitch*N
_classD
B@loc:@batch_normalization_3/moments/sufficient_statistics/mean_ss*
T0*
Tshape0
ë
Ogradients/batch_normalization_3/moments/sufficient_statistics/mean_ss_grad/TileTileRgradients/batch_normalization_3/moments/sufficient_statistics/mean_ss_grad/ReshapeSgradients/batch_normalization_3/moments/sufficient_statistics/mean_ss_grad/floordiv*

Tmultiples0*N
_classD
B@loc:@batch_normalization_3/moments/sufficient_statistics/mean_ss*
T0
÷
gradients/AddN_6AddNDgradients/batch_normalization_3/moments/normalize/Mul_grad/Reshape_1Egradients/batch_normalization_3/moments/normalize/mean_grad/Reshape_1*>
_class4
20loc:@batch_normalization_3/moments/normalize/Mul*
T0*
N
ï
Mgradients/batch_normalization_3/moments/normalize/divisor_grad/ReciprocalGradReciprocalGrad/batch_normalization_3/moments/normalize/divisorgradients/AddN_6*B
_class8
64loc:@batch_normalization_3/moments/normalize/divisor*
T0
ò
gradients/AddN_7AddNJgradients/batch_normalization_3/cond/batchnorm/mul_1/Switch_grad/cond_grad<gradients/batch_normalization_3/batchnorm/mul_1_grad/ReshapeOgradients/batch_normalization_3/moments/sufficient_statistics/Square_grad/mul_1Ogradients/batch_normalization_3/moments/sufficient_statistics/mean_ss_grad/Tile*
_class
loc:@h2/Relu*
T0*
N
k
gradients/h2/Relu_grad/ReluGradReluGradgradients/AddN_7h2/Relu*
_class
loc:@h2/Relu*
T0

%gradients/h2/BiasAdd_grad/BiasAddGradBiasAddGradgradients/h2/Relu_grad/ReluGrad*
_class
loc:@h2/BiasAdd*
T0*
data_formatNHWC
§
gradients/h2/MatMul_grad/MatMulMatMulgradients/h2/Relu_grad/ReluGradh2/kernel/read*
transpose_b(*
transpose_a( *
_class
loc:@h2/MatMul*
T0
»
!gradients/h2/MatMul_grad/MatMul_1MatMul batch_normalization_2/cond/Mergegradients/h2/Relu_grad/ReluGrad*
transpose_b( *
transpose_a(*
_class
loc:@h2/MatMul*
T0
¯
9gradients/batch_normalization_2/cond/Merge_grad/cond_gradSwitchgradients/h2/MatMul_grad/MatMul"batch_normalization_2/cond/pred_id*
_class
loc:@h2/MatMul*
T0
Ì
?gradients/batch_normalization_2/cond/batchnorm/add_1_grad/ShapeShape*batch_normalization_2/cond/batchnorm/mul_1*
out_type0*
T0*=
_class3
1/loc:@batch_normalization_2/cond/batchnorm/add_1
®
Agradients/batch_normalization_2/cond/batchnorm/add_1_grad/Shape_1Const*
dtype0*=
_class3
1/loc:@batch_normalization_2/cond/batchnorm/add_1*
valueB:d
´
Ogradients/batch_normalization_2/cond/batchnorm/add_1_grad/BroadcastGradientArgsBroadcastGradientArgs?gradients/batch_normalization_2/cond/batchnorm/add_1_grad/ShapeAgradients/batch_normalization_2/cond/batchnorm/add_1_grad/Shape_1*=
_class3
1/loc:@batch_normalization_2/cond/batchnorm/add_1*
T0
µ
=gradients/batch_normalization_2/cond/batchnorm/add_1_grad/SumSum9gradients/batch_normalization_2/cond/Merge_grad/cond_gradOgradients/batch_normalization_2/cond/batchnorm/add_1_grad/BroadcastGradientArgs*=
_class3
1/loc:@batch_normalization_2/cond/batchnorm/add_1*
T0*
	keep_dims( *

Tidx0
¢
Agradients/batch_normalization_2/cond/batchnorm/add_1_grad/ReshapeReshape=gradients/batch_normalization_2/cond/batchnorm/add_1_grad/Sum?gradients/batch_normalization_2/cond/batchnorm/add_1_grad/Shape*=
_class3
1/loc:@batch_normalization_2/cond/batchnorm/add_1*
T0*
Tshape0
¹
?gradients/batch_normalization_2/cond/batchnorm/add_1_grad/Sum_1Sum9gradients/batch_normalization_2/cond/Merge_grad/cond_gradQgradients/batch_normalization_2/cond/batchnorm/add_1_grad/BroadcastGradientArgs:1*=
_class3
1/loc:@batch_normalization_2/cond/batchnorm/add_1*
T0*
	keep_dims( *

Tidx0
¨
Cgradients/batch_normalization_2/cond/batchnorm/add_1_grad/Reshape_1Reshape?gradients/batch_normalization_2/cond/batchnorm/add_1_grad/Sum_1Agradients/batch_normalization_2/cond/batchnorm/add_1_grad/Shape_1*=
_class3
1/loc:@batch_normalization_2/cond/batchnorm/add_1*
T0*
Tshape0
ª
gradients/Switch_4Switch%batch_normalization_2/batchnorm/add_1"batch_normalization_2/cond/pred_id*8
_class.
,*loc:@batch_normalization_2/batchnorm/add_1*
T0

gradients/Shape_5Shapegradients/Switch_4*
out_type0*
T0*8
_class.
,*loc:@batch_normalization_2/batchnorm/add_1
~
gradients/zeros_4/ConstConst*
dtype0*8
_class.
,*loc:@batch_normalization_2/batchnorm/add_1*
valueB
 *    

gradients/zeros_4Fillgradients/Shape_5gradients/zeros_4/Const*8
_class.
,*loc:@batch_normalization_2/batchnorm/add_1*
T0
á
<gradients/batch_normalization_2/cond/Switch_1_grad/cond_gradMerge;gradients/batch_normalization_2/cond/Merge_grad/cond_grad:1gradients/zeros_4*8
_class.
,*loc:@batch_normalization_2/batchnorm/add_1*
T0*
N
Ó
?gradients/batch_normalization_2/cond/batchnorm/mul_1_grad/ShapeShape1batch_normalization_2/cond/batchnorm/mul_1/Switch*
out_type0*
T0*=
_class3
1/loc:@batch_normalization_2/cond/batchnorm/mul_1
®
Agradients/batch_normalization_2/cond/batchnorm/mul_1_grad/Shape_1Const*
dtype0*=
_class3
1/loc:@batch_normalization_2/cond/batchnorm/mul_1*
valueB:d
´
Ogradients/batch_normalization_2/cond/batchnorm/mul_1_grad/BroadcastGradientArgsBroadcastGradientArgs?gradients/batch_normalization_2/cond/batchnorm/mul_1_grad/ShapeAgradients/batch_normalization_2/cond/batchnorm/mul_1_grad/Shape_1*=
_class3
1/loc:@batch_normalization_2/cond/batchnorm/mul_1*
T0
ù
=gradients/batch_normalization_2/cond/batchnorm/mul_1_grad/mulMulAgradients/batch_normalization_2/cond/batchnorm/add_1_grad/Reshape(batch_normalization_2/cond/batchnorm/mul*=
_class3
1/loc:@batch_normalization_2/cond/batchnorm/mul_1*
T0
¹
=gradients/batch_normalization_2/cond/batchnorm/mul_1_grad/SumSum=gradients/batch_normalization_2/cond/batchnorm/mul_1_grad/mulOgradients/batch_normalization_2/cond/batchnorm/mul_1_grad/BroadcastGradientArgs*=
_class3
1/loc:@batch_normalization_2/cond/batchnorm/mul_1*
T0*
	keep_dims( *

Tidx0
¢
Agradients/batch_normalization_2/cond/batchnorm/mul_1_grad/ReshapeReshape=gradients/batch_normalization_2/cond/batchnorm/mul_1_grad/Sum?gradients/batch_normalization_2/cond/batchnorm/mul_1_grad/Shape*=
_class3
1/loc:@batch_normalization_2/cond/batchnorm/mul_1*
T0*
Tshape0

?gradients/batch_normalization_2/cond/batchnorm/mul_1_grad/mul_1Mul1batch_normalization_2/cond/batchnorm/mul_1/SwitchAgradients/batch_normalization_2/cond/batchnorm/add_1_grad/Reshape*=
_class3
1/loc:@batch_normalization_2/cond/batchnorm/mul_1*
T0
¿
?gradients/batch_normalization_2/cond/batchnorm/mul_1_grad/Sum_1Sum?gradients/batch_normalization_2/cond/batchnorm/mul_1_grad/mul_1Qgradients/batch_normalization_2/cond/batchnorm/mul_1_grad/BroadcastGradientArgs:1*=
_class3
1/loc:@batch_normalization_2/cond/batchnorm/mul_1*
T0*
	keep_dims( *

Tidx0
¨
Cgradients/batch_normalization_2/cond/batchnorm/mul_1_grad/Reshape_1Reshape?gradients/batch_normalization_2/cond/batchnorm/mul_1_grad/Sum_1Agradients/batch_normalization_2/cond/batchnorm/mul_1_grad/Shape_1*=
_class3
1/loc:@batch_normalization_2/cond/batchnorm/mul_1*
T0*
Tshape0
¨
=gradients/batch_normalization_2/cond/batchnorm/sub_grad/ShapeConst*
dtype0*;
_class1
/-loc:@batch_normalization_2/cond/batchnorm/sub*
valueB:d
ª
?gradients/batch_normalization_2/cond/batchnorm/sub_grad/Shape_1Const*
dtype0*;
_class1
/-loc:@batch_normalization_2/cond/batchnorm/sub*
valueB:d
¬
Mgradients/batch_normalization_2/cond/batchnorm/sub_grad/BroadcastGradientArgsBroadcastGradientArgs=gradients/batch_normalization_2/cond/batchnorm/sub_grad/Shape?gradients/batch_normalization_2/cond/batchnorm/sub_grad/Shape_1*;
_class1
/-loc:@batch_normalization_2/cond/batchnorm/sub*
T0
¹
;gradients/batch_normalization_2/cond/batchnorm/sub_grad/SumSumCgradients/batch_normalization_2/cond/batchnorm/add_1_grad/Reshape_1Mgradients/batch_normalization_2/cond/batchnorm/sub_grad/BroadcastGradientArgs*;
_class1
/-loc:@batch_normalization_2/cond/batchnorm/sub*
T0*
	keep_dims( *

Tidx0

?gradients/batch_normalization_2/cond/batchnorm/sub_grad/ReshapeReshape;gradients/batch_normalization_2/cond/batchnorm/sub_grad/Sum=gradients/batch_normalization_2/cond/batchnorm/sub_grad/Shape*;
_class1
/-loc:@batch_normalization_2/cond/batchnorm/sub*
T0*
Tshape0
½
=gradients/batch_normalization_2/cond/batchnorm/sub_grad/Sum_1SumCgradients/batch_normalization_2/cond/batchnorm/add_1_grad/Reshape_1Ogradients/batch_normalization_2/cond/batchnorm/sub_grad/BroadcastGradientArgs:1*;
_class1
/-loc:@batch_normalization_2/cond/batchnorm/sub*
T0*
	keep_dims( *

Tidx0
Ç
;gradients/batch_normalization_2/cond/batchnorm/sub_grad/NegNeg=gradients/batch_normalization_2/cond/batchnorm/sub_grad/Sum_1*;
_class1
/-loc:@batch_normalization_2/cond/batchnorm/sub*
T0

Agradients/batch_normalization_2/cond/batchnorm/sub_grad/Reshape_1Reshape;gradients/batch_normalization_2/cond/batchnorm/sub_grad/Neg?gradients/batch_normalization_2/cond/batchnorm/sub_grad/Shape_1*;
_class1
/-loc:@batch_normalization_2/cond/batchnorm/sub*
T0*
Tshape0
½
:gradients/batch_normalization_2/batchnorm/add_1_grad/ShapeShape%batch_normalization_2/batchnorm/mul_1*
out_type0*
T0*8
_class.
,*loc:@batch_normalization_2/batchnorm/add_1
¤
<gradients/batch_normalization_2/batchnorm/add_1_grad/Shape_1Const*
dtype0*8
_class.
,*loc:@batch_normalization_2/batchnorm/add_1*
valueB:d
 
Jgradients/batch_normalization_2/batchnorm/add_1_grad/BroadcastGradientArgsBroadcastGradientArgs:gradients/batch_normalization_2/batchnorm/add_1_grad/Shape<gradients/batch_normalization_2/batchnorm/add_1_grad/Shape_1*8
_class.
,*loc:@batch_normalization_2/batchnorm/add_1*
T0
©
8gradients/batch_normalization_2/batchnorm/add_1_grad/SumSum<gradients/batch_normalization_2/cond/Switch_1_grad/cond_gradJgradients/batch_normalization_2/batchnorm/add_1_grad/BroadcastGradientArgs*8
_class.
,*loc:@batch_normalization_2/batchnorm/add_1*
T0*
	keep_dims( *

Tidx0

<gradients/batch_normalization_2/batchnorm/add_1_grad/ReshapeReshape8gradients/batch_normalization_2/batchnorm/add_1_grad/Sum:gradients/batch_normalization_2/batchnorm/add_1_grad/Shape*8
_class.
,*loc:@batch_normalization_2/batchnorm/add_1*
T0*
Tshape0
­
:gradients/batch_normalization_2/batchnorm/add_1_grad/Sum_1Sum<gradients/batch_normalization_2/cond/Switch_1_grad/cond_gradLgradients/batch_normalization_2/batchnorm/add_1_grad/BroadcastGradientArgs:1*8
_class.
,*loc:@batch_normalization_2/batchnorm/add_1*
T0*
	keep_dims( *

Tidx0

>gradients/batch_normalization_2/batchnorm/add_1_grad/Reshape_1Reshape:gradients/batch_normalization_2/batchnorm/add_1_grad/Sum_1<gradients/batch_normalization_2/batchnorm/add_1_grad/Shape_1*8
_class.
,*loc:@batch_normalization_2/batchnorm/add_1*
T0*
Tshape0
n
gradients/Switch_5Switchh1/Relu"batch_normalization_2/cond/pred_id*
_class
loc:@h1/Relu*
T0
e
gradients/Shape_6Shapegradients/Switch_5:1*
out_type0*
T0*
_class
loc:@h1/Relu
`
gradients/zeros_5/ConstConst*
dtype0*
_class
loc:@h1/Relu*
valueB
 *    
j
gradients/zeros_5Fillgradients/Shape_6gradients/zeros_5/Const*
_class
loc:@h1/Relu*
T0
×
Jgradients/batch_normalization_2/cond/batchnorm/mul_1/Switch_grad/cond_gradMergeAgradients/batch_normalization_2/cond/batchnorm/mul_1_grad/Reshapegradients/zeros_5*
_class
loc:@h1/Relu*
T0*
N

gradients/Switch_6Switchbatch_normalization_2/beta/read"batch_normalization_2/cond/pred_id*-
_class#
!loc:@batch_normalization_2/beta*
T0
x
gradients/Shape_7Shapegradients/Switch_6:1*
out_type0*
T0*-
_class#
!loc:@batch_normalization_2/beta
s
gradients/zeros_6/ConstConst*
dtype0*-
_class#
!loc:@batch_normalization_2/beta*
valueB
 *    
}
gradients/zeros_6Fillgradients/Shape_7gradients/zeros_6/Const*-
_class#
!loc:@batch_normalization_2/beta*
T0
æ
Hgradients/batch_normalization_2/cond/batchnorm/sub/Switch_grad/cond_gradMerge?gradients/batch_normalization_2/cond/batchnorm/sub_grad/Reshapegradients/zeros_6*-
_class#
!loc:@batch_normalization_2/beta*
T0*
N
¬
?gradients/batch_normalization_2/cond/batchnorm/mul_2_grad/ShapeConst*
dtype0*=
_class3
1/loc:@batch_normalization_2/cond/batchnorm/mul_2*
valueB:d
®
Agradients/batch_normalization_2/cond/batchnorm/mul_2_grad/Shape_1Const*
dtype0*=
_class3
1/loc:@batch_normalization_2/cond/batchnorm/mul_2*
valueB:d
´
Ogradients/batch_normalization_2/cond/batchnorm/mul_2_grad/BroadcastGradientArgsBroadcastGradientArgs?gradients/batch_normalization_2/cond/batchnorm/mul_2_grad/ShapeAgradients/batch_normalization_2/cond/batchnorm/mul_2_grad/Shape_1*=
_class3
1/loc:@batch_normalization_2/cond/batchnorm/mul_2*
T0
ù
=gradients/batch_normalization_2/cond/batchnorm/mul_2_grad/mulMulAgradients/batch_normalization_2/cond/batchnorm/sub_grad/Reshape_1(batch_normalization_2/cond/batchnorm/mul*=
_class3
1/loc:@batch_normalization_2/cond/batchnorm/mul_2*
T0
¹
=gradients/batch_normalization_2/cond/batchnorm/mul_2_grad/SumSum=gradients/batch_normalization_2/cond/batchnorm/mul_2_grad/mulOgradients/batch_normalization_2/cond/batchnorm/mul_2_grad/BroadcastGradientArgs*=
_class3
1/loc:@batch_normalization_2/cond/batchnorm/mul_2*
T0*
	keep_dims( *

Tidx0
¢
Agradients/batch_normalization_2/cond/batchnorm/mul_2_grad/ReshapeReshape=gradients/batch_normalization_2/cond/batchnorm/mul_2_grad/Sum?gradients/batch_normalization_2/cond/batchnorm/mul_2_grad/Shape*=
_class3
1/loc:@batch_normalization_2/cond/batchnorm/mul_2*
T0*
Tshape0

?gradients/batch_normalization_2/cond/batchnorm/mul_2_grad/mul_1Mul1batch_normalization_2/cond/batchnorm/mul_2/SwitchAgradients/batch_normalization_2/cond/batchnorm/sub_grad/Reshape_1*=
_class3
1/loc:@batch_normalization_2/cond/batchnorm/mul_2*
T0
¿
?gradients/batch_normalization_2/cond/batchnorm/mul_2_grad/Sum_1Sum?gradients/batch_normalization_2/cond/batchnorm/mul_2_grad/mul_1Qgradients/batch_normalization_2/cond/batchnorm/mul_2_grad/BroadcastGradientArgs:1*=
_class3
1/loc:@batch_normalization_2/cond/batchnorm/mul_2*
T0*
	keep_dims( *

Tidx0
¨
Cgradients/batch_normalization_2/cond/batchnorm/mul_2_grad/Reshape_1Reshape?gradients/batch_normalization_2/cond/batchnorm/mul_2_grad/Sum_1Agradients/batch_normalization_2/cond/batchnorm/mul_2_grad/Shape_1*=
_class3
1/loc:@batch_normalization_2/cond/batchnorm/mul_2*
T0*
Tshape0

:gradients/batch_normalization_2/batchnorm/mul_1_grad/ShapeShapeh1/Relu*
out_type0*
T0*8
_class.
,*loc:@batch_normalization_2/batchnorm/mul_1
¤
<gradients/batch_normalization_2/batchnorm/mul_1_grad/Shape_1Const*
dtype0*8
_class.
,*loc:@batch_normalization_2/batchnorm/mul_1*
valueB:d
 
Jgradients/batch_normalization_2/batchnorm/mul_1_grad/BroadcastGradientArgsBroadcastGradientArgs:gradients/batch_normalization_2/batchnorm/mul_1_grad/Shape<gradients/batch_normalization_2/batchnorm/mul_1_grad/Shape_1*8
_class.
,*loc:@batch_normalization_2/batchnorm/mul_1*
T0
å
8gradients/batch_normalization_2/batchnorm/mul_1_grad/mulMul<gradients/batch_normalization_2/batchnorm/add_1_grad/Reshape#batch_normalization_2/batchnorm/mul*8
_class.
,*loc:@batch_normalization_2/batchnorm/mul_1*
T0
¥
8gradients/batch_normalization_2/batchnorm/mul_1_grad/SumSum8gradients/batch_normalization_2/batchnorm/mul_1_grad/mulJgradients/batch_normalization_2/batchnorm/mul_1_grad/BroadcastGradientArgs*8
_class.
,*loc:@batch_normalization_2/batchnorm/mul_1*
T0*
	keep_dims( *

Tidx0

<gradients/batch_normalization_2/batchnorm/mul_1_grad/ReshapeReshape8gradients/batch_normalization_2/batchnorm/mul_1_grad/Sum:gradients/batch_normalization_2/batchnorm/mul_1_grad/Shape*8
_class.
,*loc:@batch_normalization_2/batchnorm/mul_1*
T0*
Tshape0
Ë
:gradients/batch_normalization_2/batchnorm/mul_1_grad/mul_1Mulh1/Relu<gradients/batch_normalization_2/batchnorm/add_1_grad/Reshape*8
_class.
,*loc:@batch_normalization_2/batchnorm/mul_1*
T0
«
:gradients/batch_normalization_2/batchnorm/mul_1_grad/Sum_1Sum:gradients/batch_normalization_2/batchnorm/mul_1_grad/mul_1Lgradients/batch_normalization_2/batchnorm/mul_1_grad/BroadcastGradientArgs:1*8
_class.
,*loc:@batch_normalization_2/batchnorm/mul_1*
T0*
	keep_dims( *

Tidx0

>gradients/batch_normalization_2/batchnorm/mul_1_grad/Reshape_1Reshape:gradients/batch_normalization_2/batchnorm/mul_1_grad/Sum_1<gradients/batch_normalization_2/batchnorm/mul_1_grad/Shape_1*8
_class.
,*loc:@batch_normalization_2/batchnorm/mul_1*
T0*
Tshape0

8gradients/batch_normalization_2/batchnorm/sub_grad/ShapeConst*
dtype0*6
_class,
*(loc:@batch_normalization_2/batchnorm/sub*
valueB:d
 
:gradients/batch_normalization_2/batchnorm/sub_grad/Shape_1Const*
dtype0*6
_class,
*(loc:@batch_normalization_2/batchnorm/sub*
valueB:d

Hgradients/batch_normalization_2/batchnorm/sub_grad/BroadcastGradientArgsBroadcastGradientArgs8gradients/batch_normalization_2/batchnorm/sub_grad/Shape:gradients/batch_normalization_2/batchnorm/sub_grad/Shape_1*6
_class,
*(loc:@batch_normalization_2/batchnorm/sub*
T0
¥
6gradients/batch_normalization_2/batchnorm/sub_grad/SumSum>gradients/batch_normalization_2/batchnorm/add_1_grad/Reshape_1Hgradients/batch_normalization_2/batchnorm/sub_grad/BroadcastGradientArgs*6
_class,
*(loc:@batch_normalization_2/batchnorm/sub*
T0*
	keep_dims( *

Tidx0

:gradients/batch_normalization_2/batchnorm/sub_grad/ReshapeReshape6gradients/batch_normalization_2/batchnorm/sub_grad/Sum8gradients/batch_normalization_2/batchnorm/sub_grad/Shape*6
_class,
*(loc:@batch_normalization_2/batchnorm/sub*
T0*
Tshape0
©
8gradients/batch_normalization_2/batchnorm/sub_grad/Sum_1Sum>gradients/batch_normalization_2/batchnorm/add_1_grad/Reshape_1Jgradients/batch_normalization_2/batchnorm/sub_grad/BroadcastGradientArgs:1*6
_class,
*(loc:@batch_normalization_2/batchnorm/sub*
T0*
	keep_dims( *

Tidx0
¸
6gradients/batch_normalization_2/batchnorm/sub_grad/NegNeg8gradients/batch_normalization_2/batchnorm/sub_grad/Sum_1*6
_class,
*(loc:@batch_normalization_2/batchnorm/sub*
T0

<gradients/batch_normalization_2/batchnorm/sub_grad/Reshape_1Reshape6gradients/batch_normalization_2/batchnorm/sub_grad/Neg:gradients/batch_normalization_2/batchnorm/sub_grad/Shape_1*6
_class,
*(loc:@batch_normalization_2/batchnorm/sub*
T0*
Tshape0
ó
gradients/AddN_8AddNCgradients/batch_normalization_2/cond/batchnorm/mul_1_grad/Reshape_1Cgradients/batch_normalization_2/cond/batchnorm/mul_2_grad/Reshape_1*=
_class3
1/loc:@batch_normalization_2/cond/batchnorm/mul_1*
T0*
N
¨
=gradients/batch_normalization_2/cond/batchnorm/mul_grad/ShapeConst*
dtype0*;
_class1
/-loc:@batch_normalization_2/cond/batchnorm/mul*
valueB:d
ª
?gradients/batch_normalization_2/cond/batchnorm/mul_grad/Shape_1Const*
dtype0*;
_class1
/-loc:@batch_normalization_2/cond/batchnorm/mul*
valueB:d
¬
Mgradients/batch_normalization_2/cond/batchnorm/mul_grad/BroadcastGradientArgsBroadcastGradientArgs=gradients/batch_normalization_2/cond/batchnorm/mul_grad/Shape?gradients/batch_normalization_2/cond/batchnorm/mul_grad/Shape_1*;
_class1
/-loc:@batch_normalization_2/cond/batchnorm/mul*
T0
Ë
;gradients/batch_normalization_2/cond/batchnorm/mul_grad/mulMulgradients/AddN_8/batch_normalization_2/cond/batchnorm/mul/Switch*;
_class1
/-loc:@batch_normalization_2/cond/batchnorm/mul*
T0
±
;gradients/batch_normalization_2/cond/batchnorm/mul_grad/SumSum;gradients/batch_normalization_2/cond/batchnorm/mul_grad/mulMgradients/batch_normalization_2/cond/batchnorm/mul_grad/BroadcastGradientArgs*;
_class1
/-loc:@batch_normalization_2/cond/batchnorm/mul*
T0*
	keep_dims( *

Tidx0

?gradients/batch_normalization_2/cond/batchnorm/mul_grad/ReshapeReshape;gradients/batch_normalization_2/cond/batchnorm/mul_grad/Sum=gradients/batch_normalization_2/cond/batchnorm/mul_grad/Shape*;
_class1
/-loc:@batch_normalization_2/cond/batchnorm/mul*
T0*
Tshape0
È
=gradients/batch_normalization_2/cond/batchnorm/mul_grad/mul_1Mul*batch_normalization_2/cond/batchnorm/Rsqrtgradients/AddN_8*;
_class1
/-loc:@batch_normalization_2/cond/batchnorm/mul*
T0
·
=gradients/batch_normalization_2/cond/batchnorm/mul_grad/Sum_1Sum=gradients/batch_normalization_2/cond/batchnorm/mul_grad/mul_1Ogradients/batch_normalization_2/cond/batchnorm/mul_grad/BroadcastGradientArgs:1*;
_class1
/-loc:@batch_normalization_2/cond/batchnorm/mul*
T0*
	keep_dims( *

Tidx0
 
Agradients/batch_normalization_2/cond/batchnorm/mul_grad/Reshape_1Reshape=gradients/batch_normalization_2/cond/batchnorm/mul_grad/Sum_1?gradients/batch_normalization_2/cond/batchnorm/mul_grad/Shape_1*;
_class1
/-loc:@batch_normalization_2/cond/batchnorm/mul*
T0*
Tshape0
ß
gradients/AddN_9AddNHgradients/batch_normalization_2/cond/batchnorm/sub/Switch_grad/cond_grad:gradients/batch_normalization_2/batchnorm/sub_grad/Reshape*-
_class#
!loc:@batch_normalization_2/beta*
T0*
N
¢
:gradients/batch_normalization_2/batchnorm/mul_2_grad/ShapeConst*
dtype0*8
_class.
,*loc:@batch_normalization_2/batchnorm/mul_2*
valueB:d
¤
<gradients/batch_normalization_2/batchnorm/mul_2_grad/Shape_1Const*
dtype0*8
_class.
,*loc:@batch_normalization_2/batchnorm/mul_2*
valueB:d
 
Jgradients/batch_normalization_2/batchnorm/mul_2_grad/BroadcastGradientArgsBroadcastGradientArgs:gradients/batch_normalization_2/batchnorm/mul_2_grad/Shape<gradients/batch_normalization_2/batchnorm/mul_2_grad/Shape_1*8
_class.
,*loc:@batch_normalization_2/batchnorm/mul_2*
T0
å
8gradients/batch_normalization_2/batchnorm/mul_2_grad/mulMul<gradients/batch_normalization_2/batchnorm/sub_grad/Reshape_1#batch_normalization_2/batchnorm/mul*8
_class.
,*loc:@batch_normalization_2/batchnorm/mul_2*
T0
¥
8gradients/batch_normalization_2/batchnorm/mul_2_grad/SumSum8gradients/batch_normalization_2/batchnorm/mul_2_grad/mulJgradients/batch_normalization_2/batchnorm/mul_2_grad/BroadcastGradientArgs*8
_class.
,*loc:@batch_normalization_2/batchnorm/mul_2*
T0*
	keep_dims( *

Tidx0

<gradients/batch_normalization_2/batchnorm/mul_2_grad/ReshapeReshape8gradients/batch_normalization_2/batchnorm/mul_2_grad/Sum:gradients/batch_normalization_2/batchnorm/mul_2_grad/Shape*8
_class.
,*loc:@batch_normalization_2/batchnorm/mul_2*
T0*
Tshape0
ð
:gradients/batch_normalization_2/batchnorm/mul_2_grad/mul_1Mul,batch_normalization_2/moments/normalize/mean<gradients/batch_normalization_2/batchnorm/sub_grad/Reshape_1*8
_class.
,*loc:@batch_normalization_2/batchnorm/mul_2*
T0
«
:gradients/batch_normalization_2/batchnorm/mul_2_grad/Sum_1Sum:gradients/batch_normalization_2/batchnorm/mul_2_grad/mul_1Lgradients/batch_normalization_2/batchnorm/mul_2_grad/BroadcastGradientArgs:1*8
_class.
,*loc:@batch_normalization_2/batchnorm/mul_2*
T0*
	keep_dims( *

Tidx0

>gradients/batch_normalization_2/batchnorm/mul_2_grad/Reshape_1Reshape:gradients/batch_normalization_2/batchnorm/mul_2_grad/Sum_1<gradients/batch_normalization_2/batchnorm/mul_2_grad/Shape_1*8
_class.
,*loc:@batch_normalization_2/batchnorm/mul_2*
T0*
Tshape0

gradients/Switch_7Switch batch_normalization_2/gamma/read"batch_normalization_2/cond/pred_id*.
_class$
" loc:@batch_normalization_2/gamma*
T0
y
gradients/Shape_8Shapegradients/Switch_7:1*
out_type0*
T0*.
_class$
" loc:@batch_normalization_2/gamma
t
gradients/zeros_7/ConstConst*
dtype0*.
_class$
" loc:@batch_normalization_2/gamma*
valueB
 *    
~
gradients/zeros_7Fillgradients/Shape_8gradients/zeros_7/Const*.
_class$
" loc:@batch_normalization_2/gamma*
T0
é
Hgradients/batch_normalization_2/cond/batchnorm/mul/Switch_grad/cond_gradMergeAgradients/batch_normalization_2/cond/batchnorm/mul_grad/Reshape_1gradients/zeros_7*.
_class$
" loc:@batch_normalization_2/gamma*
T0*
N
å
gradients/AddN_10AddN>gradients/batch_normalization_2/batchnorm/mul_1_grad/Reshape_1>gradients/batch_normalization_2/batchnorm/mul_2_grad/Reshape_1*8
_class.
,*loc:@batch_normalization_2/batchnorm/mul_1*
T0*
N

8gradients/batch_normalization_2/batchnorm/mul_grad/ShapeConst*
dtype0*6
_class,
*(loc:@batch_normalization_2/batchnorm/mul*
valueB:d
 
:gradients/batch_normalization_2/batchnorm/mul_grad/Shape_1Const*
dtype0*6
_class,
*(loc:@batch_normalization_2/batchnorm/mul*
valueB:d

Hgradients/batch_normalization_2/batchnorm/mul_grad/BroadcastGradientArgsBroadcastGradientArgs8gradients/batch_normalization_2/batchnorm/mul_grad/Shape:gradients/batch_normalization_2/batchnorm/mul_grad/Shape_1*6
_class,
*(loc:@batch_normalization_2/batchnorm/mul*
T0
³
6gradients/batch_normalization_2/batchnorm/mul_grad/mulMulgradients/AddN_10 batch_normalization_2/gamma/read*6
_class,
*(loc:@batch_normalization_2/batchnorm/mul*
T0

6gradients/batch_normalization_2/batchnorm/mul_grad/SumSum6gradients/batch_normalization_2/batchnorm/mul_grad/mulHgradients/batch_normalization_2/batchnorm/mul_grad/BroadcastGradientArgs*6
_class,
*(loc:@batch_normalization_2/batchnorm/mul*
T0*
	keep_dims( *

Tidx0

:gradients/batch_normalization_2/batchnorm/mul_grad/ReshapeReshape6gradients/batch_normalization_2/batchnorm/mul_grad/Sum8gradients/batch_normalization_2/batchnorm/mul_grad/Shape*6
_class,
*(loc:@batch_normalization_2/batchnorm/mul*
T0*
Tshape0
º
8gradients/batch_normalization_2/batchnorm/mul_grad/mul_1Mul%batch_normalization_2/batchnorm/Rsqrtgradients/AddN_10*6
_class,
*(loc:@batch_normalization_2/batchnorm/mul*
T0
£
8gradients/batch_normalization_2/batchnorm/mul_grad/Sum_1Sum8gradients/batch_normalization_2/batchnorm/mul_grad/mul_1Jgradients/batch_normalization_2/batchnorm/mul_grad/BroadcastGradientArgs:1*6
_class,
*(loc:@batch_normalization_2/batchnorm/mul*
T0*
	keep_dims( *

Tidx0

<gradients/batch_normalization_2/batchnorm/mul_grad/Reshape_1Reshape8gradients/batch_normalization_2/batchnorm/mul_grad/Sum_1:gradients/batch_normalization_2/batchnorm/mul_grad/Shape_1*6
_class,
*(loc:@batch_normalization_2/batchnorm/mul*
T0*
Tshape0
ñ
>gradients/batch_normalization_2/batchnorm/Rsqrt_grad/RsqrtGrad	RsqrtGrad%batch_normalization_2/batchnorm/Rsqrt:gradients/batch_normalization_2/batchnorm/mul_grad/Reshape*8
_class.
,*loc:@batch_normalization_2/batchnorm/Rsqrt*
T0
ã
gradients/AddN_11AddNHgradients/batch_normalization_2/cond/batchnorm/mul/Switch_grad/cond_grad<gradients/batch_normalization_2/batchnorm/mul_grad/Reshape_1*.
_class$
" loc:@batch_normalization_2/gamma*
T0*
N

8gradients/batch_normalization_2/batchnorm/add_grad/ShapeConst*
dtype0*6
_class,
*(loc:@batch_normalization_2/batchnorm/add*
valueB:d

:gradients/batch_normalization_2/batchnorm/add_grad/Shape_1Const*
dtype0*6
_class,
*(loc:@batch_normalization_2/batchnorm/add*
valueB 

Hgradients/batch_normalization_2/batchnorm/add_grad/BroadcastGradientArgsBroadcastGradientArgs8gradients/batch_normalization_2/batchnorm/add_grad/Shape:gradients/batch_normalization_2/batchnorm/add_grad/Shape_1*6
_class,
*(loc:@batch_normalization_2/batchnorm/add*
T0
¥
6gradients/batch_normalization_2/batchnorm/add_grad/SumSum>gradients/batch_normalization_2/batchnorm/Rsqrt_grad/RsqrtGradHgradients/batch_normalization_2/batchnorm/add_grad/BroadcastGradientArgs*6
_class,
*(loc:@batch_normalization_2/batchnorm/add*
T0*
	keep_dims( *

Tidx0

:gradients/batch_normalization_2/batchnorm/add_grad/ReshapeReshape6gradients/batch_normalization_2/batchnorm/add_grad/Sum8gradients/batch_normalization_2/batchnorm/add_grad/Shape*6
_class,
*(loc:@batch_normalization_2/batchnorm/add*
T0*
Tshape0
©
8gradients/batch_normalization_2/batchnorm/add_grad/Sum_1Sum>gradients/batch_normalization_2/batchnorm/Rsqrt_grad/RsqrtGradJgradients/batch_normalization_2/batchnorm/add_grad/BroadcastGradientArgs:1*6
_class,
*(loc:@batch_normalization_2/batchnorm/add*
T0*
	keep_dims( *

Tidx0

<gradients/batch_normalization_2/batchnorm/add_grad/Reshape_1Reshape8gradients/batch_normalization_2/batchnorm/add_grad/Sum_1:gradients/batch_normalization_2/batchnorm/add_grad/Shape_1*6
_class,
*(loc:@batch_normalization_2/batchnorm/add*
T0*
Tshape0
¸
Egradients/batch_normalization_2/moments/normalize/variance_grad/ShapeConst*
dtype0*C
_class9
75loc:@batch_normalization_2/moments/normalize/variance*
valueB:d
º
Ggradients/batch_normalization_2/moments/normalize/variance_grad/Shape_1Const*
dtype0*C
_class9
75loc:@batch_normalization_2/moments/normalize/variance*
valueB:d
Ì
Ugradients/batch_normalization_2/moments/normalize/variance_grad/BroadcastGradientArgsBroadcastGradientArgsEgradients/batch_normalization_2/moments/normalize/variance_grad/ShapeGgradients/batch_normalization_2/moments/normalize/variance_grad/Shape_1*C
_class9
75loc:@batch_normalization_2/moments/normalize/variance*
T0
È
Cgradients/batch_normalization_2/moments/normalize/variance_grad/SumSum:gradients/batch_normalization_2/batchnorm/add_grad/ReshapeUgradients/batch_normalization_2/moments/normalize/variance_grad/BroadcastGradientArgs*C
_class9
75loc:@batch_normalization_2/moments/normalize/variance*
T0*
	keep_dims( *

Tidx0
º
Ggradients/batch_normalization_2/moments/normalize/variance_grad/ReshapeReshapeCgradients/batch_normalization_2/moments/normalize/variance_grad/SumEgradients/batch_normalization_2/moments/normalize/variance_grad/Shape*C
_class9
75loc:@batch_normalization_2/moments/normalize/variance*
T0*
Tshape0
Ì
Egradients/batch_normalization_2/moments/normalize/variance_grad/Sum_1Sum:gradients/batch_normalization_2/batchnorm/add_grad/ReshapeWgradients/batch_normalization_2/moments/normalize/variance_grad/BroadcastGradientArgs:1*C
_class9
75loc:@batch_normalization_2/moments/normalize/variance*
T0*
	keep_dims( *

Tidx0
ß
Cgradients/batch_normalization_2/moments/normalize/variance_grad/NegNegEgradients/batch_normalization_2/moments/normalize/variance_grad/Sum_1*C
_class9
75loc:@batch_normalization_2/moments/normalize/variance*
T0
¾
Igradients/batch_normalization_2/moments/normalize/variance_grad/Reshape_1ReshapeCgradients/batch_normalization_2/moments/normalize/variance_grad/NegGgradients/batch_normalization_2/moments/normalize/variance_grad/Shape_1*C
_class9
75loc:@batch_normalization_2/moments/normalize/variance*
T0*
Tshape0
®
@gradients/batch_normalization_2/moments/normalize/Mul_grad/ShapeConst*
dtype0*>
_class4
20loc:@batch_normalization_2/moments/normalize/Mul*
valueB:d
«
Bgradients/batch_normalization_2/moments/normalize/Mul_grad/Shape_1Const*
dtype0*>
_class4
20loc:@batch_normalization_2/moments/normalize/Mul*
valueB 
¸
Pgradients/batch_normalization_2/moments/normalize/Mul_grad/BroadcastGradientArgsBroadcastGradientArgs@gradients/batch_normalization_2/moments/normalize/Mul_grad/ShapeBgradients/batch_normalization_2/moments/normalize/Mul_grad/Shape_1*>
_class4
20loc:@batch_normalization_2/moments/normalize/Mul*
T0

>gradients/batch_normalization_2/moments/normalize/Mul_grad/mulMulGgradients/batch_normalization_2/moments/normalize/variance_grad/Reshape/batch_normalization_2/moments/normalize/divisor*>
_class4
20loc:@batch_normalization_2/moments/normalize/Mul*
T0
½
>gradients/batch_normalization_2/moments/normalize/Mul_grad/SumSum>gradients/batch_normalization_2/moments/normalize/Mul_grad/mulPgradients/batch_normalization_2/moments/normalize/Mul_grad/BroadcastGradientArgs*>
_class4
20loc:@batch_normalization_2/moments/normalize/Mul*
T0*
	keep_dims( *

Tidx0
¦
Bgradients/batch_normalization_2/moments/normalize/Mul_grad/ReshapeReshape>gradients/batch_normalization_2/moments/normalize/Mul_grad/Sum@gradients/batch_normalization_2/moments/normalize/Mul_grad/Shape*>
_class4
20loc:@batch_normalization_2/moments/normalize/Mul*
T0*
Tshape0

@gradients/batch_normalization_2/moments/normalize/Mul_grad/mul_1Mul:batch_normalization_2/moments/sufficient_statistics/var_ssGgradients/batch_normalization_2/moments/normalize/variance_grad/Reshape*>
_class4
20loc:@batch_normalization_2/moments/normalize/Mul*
T0
Ã
@gradients/batch_normalization_2/moments/normalize/Mul_grad/Sum_1Sum@gradients/batch_normalization_2/moments/normalize/Mul_grad/mul_1Rgradients/batch_normalization_2/moments/normalize/Mul_grad/BroadcastGradientArgs:1*>
_class4
20loc:@batch_normalization_2/moments/normalize/Mul*
T0*
	keep_dims( *

Tidx0
¬
Dgradients/batch_normalization_2/moments/normalize/Mul_grad/Reshape_1Reshape@gradients/batch_normalization_2/moments/normalize/Mul_grad/Sum_1Bgradients/batch_normalization_2/moments/normalize/Mul_grad/Shape_1*>
_class4
20loc:@batch_normalization_2/moments/normalize/Mul*
T0*
Tshape0
ÿ
Cgradients/batch_normalization_2/moments/normalize/Square_grad/mul/xConstJ^gradients/batch_normalization_2/moments/normalize/variance_grad/Reshape_1*
dtype0*A
_class7
53loc:@batch_normalization_2/moments/normalize/Square*
valueB
 *   @

Agradients/batch_normalization_2/moments/normalize/Square_grad/mulMulCgradients/batch_normalization_2/moments/normalize/Square_grad/mul/x,batch_normalization_2/moments/normalize/mean*A
_class7
53loc:@batch_normalization_2/moments/normalize/Square*
T0
¤
Cgradients/batch_normalization_2/moments/normalize/Square_grad/mul_1MulIgradients/batch_normalization_2/moments/normalize/variance_grad/Reshape_1Agradients/batch_normalization_2/moments/normalize/Square_grad/mul*A
_class7
53loc:@batch_normalization_2/moments/normalize/Square*
T0
ü
Ogradients/batch_normalization_2/moments/sufficient_statistics/var_ss_grad/ShapeShape:batch_normalization_2/moments/sufficient_statistics/Square*
out_type0*
T0*M
_classC
A?loc:@batch_normalization_2/moments/sufficient_statistics/var_ss
Ç
Ngradients/batch_normalization_2/moments/sufficient_statistics/var_ss_grad/SizeConst*
dtype0*M
_classC
A?loc:@batch_normalization_2/moments/sufficient_statistics/var_ss*
value	B :
Ê
Mgradients/batch_normalization_2/moments/sufficient_statistics/var_ss_grad/addAddLbatch_normalization_2/moments/sufficient_statistics/var_ss/reduction_indicesNgradients/batch_normalization_2/moments/sufficient_statistics/var_ss_grad/Size*M
_classC
A?loc:@batch_normalization_2/moments/sufficient_statistics/var_ss*
T0
Ë
Mgradients/batch_normalization_2/moments/sufficient_statistics/var_ss_grad/modModMgradients/batch_normalization_2/moments/sufficient_statistics/var_ss_grad/addNgradients/batch_normalization_2/moments/sufficient_statistics/var_ss_grad/Size*M
_classC
A?loc:@batch_normalization_2/moments/sufficient_statistics/var_ss*
T0
Î
Qgradients/batch_normalization_2/moments/sufficient_statistics/var_ss_grad/Shape_1Const*
dtype0*M
_classC
A?loc:@batch_normalization_2/moments/sufficient_statistics/var_ss*
valueB:
Î
Ugradients/batch_normalization_2/moments/sufficient_statistics/var_ss_grad/range/startConst*
dtype0*M
_classC
A?loc:@batch_normalization_2/moments/sufficient_statistics/var_ss*
value	B : 
Î
Ugradients/batch_normalization_2/moments/sufficient_statistics/var_ss_grad/range/deltaConst*
dtype0*M
_classC
A?loc:@batch_normalization_2/moments/sufficient_statistics/var_ss*
value	B :
±
Ogradients/batch_normalization_2/moments/sufficient_statistics/var_ss_grad/rangeRangeUgradients/batch_normalization_2/moments/sufficient_statistics/var_ss_grad/range/startNgradients/batch_normalization_2/moments/sufficient_statistics/var_ss_grad/SizeUgradients/batch_normalization_2/moments/sufficient_statistics/var_ss_grad/range/delta*M
_classC
A?loc:@batch_normalization_2/moments/sufficient_statistics/var_ss*

Tidx0
Í
Tgradients/batch_normalization_2/moments/sufficient_statistics/var_ss_grad/Fill/valueConst*
dtype0*M
_classC
A?loc:@batch_normalization_2/moments/sufficient_statistics/var_ss*
value	B :
×
Ngradients/batch_normalization_2/moments/sufficient_statistics/var_ss_grad/FillFillQgradients/batch_normalization_2/moments/sufficient_statistics/var_ss_grad/Shape_1Tgradients/batch_normalization_2/moments/sufficient_statistics/var_ss_grad/Fill/value*M
_classC
A?loc:@batch_normalization_2/moments/sufficient_statistics/var_ss*
T0

Wgradients/batch_normalization_2/moments/sufficient_statistics/var_ss_grad/DynamicStitchDynamicStitchOgradients/batch_normalization_2/moments/sufficient_statistics/var_ss_grad/rangeMgradients/batch_normalization_2/moments/sufficient_statistics/var_ss_grad/modOgradients/batch_normalization_2/moments/sufficient_statistics/var_ss_grad/ShapeNgradients/batch_normalization_2/moments/sufficient_statistics/var_ss_grad/Fill*M
_classC
A?loc:@batch_normalization_2/moments/sufficient_statistics/var_ss*
T0*
N
Ì
Sgradients/batch_normalization_2/moments/sufficient_statistics/var_ss_grad/Maximum/yConst*
dtype0*M
_classC
A?loc:@batch_normalization_2/moments/sufficient_statistics/var_ss*
value	B :
â
Qgradients/batch_normalization_2/moments/sufficient_statistics/var_ss_grad/MaximumMaximumWgradients/batch_normalization_2/moments/sufficient_statistics/var_ss_grad/DynamicStitchSgradients/batch_normalization_2/moments/sufficient_statistics/var_ss_grad/Maximum/y*M
_classC
A?loc:@batch_normalization_2/moments/sufficient_statistics/var_ss*
T0
Õ
Rgradients/batch_normalization_2/moments/sufficient_statistics/var_ss_grad/floordivDivOgradients/batch_normalization_2/moments/sufficient_statistics/var_ss_grad/ShapeQgradients/batch_normalization_2/moments/sufficient_statistics/var_ss_grad/Maximum*M
_classC
A?loc:@batch_normalization_2/moments/sufficient_statistics/var_ss*
T0
ß
Qgradients/batch_normalization_2/moments/sufficient_statistics/var_ss_grad/ReshapeReshapeBgradients/batch_normalization_2/moments/normalize/Mul_grad/ReshapeWgradients/batch_normalization_2/moments/sufficient_statistics/var_ss_grad/DynamicStitch*M
_classC
A?loc:@batch_normalization_2/moments/sufficient_statistics/var_ss*
T0*
Tshape0
ç
Ngradients/batch_normalization_2/moments/sufficient_statistics/var_ss_grad/TileTileQgradients/batch_normalization_2/moments/sufficient_statistics/var_ss_grad/ReshapeRgradients/batch_normalization_2/moments/sufficient_statistics/var_ss_grad/floordiv*

Tmultiples0*M
_classC
A?loc:@batch_normalization_2/moments/sufficient_statistics/var_ss*
T0
è
gradients/AddN_12AddN<gradients/batch_normalization_2/batchnorm/mul_2_grad/ReshapeCgradients/batch_normalization_2/moments/normalize/Square_grad/mul_1*8
_class.
,*loc:@batch_normalization_2/batchnorm/mul_2*
T0*
N
°
Agradients/batch_normalization_2/moments/normalize/mean_grad/ShapeConst*
dtype0*?
_class5
31loc:@batch_normalization_2/moments/normalize/mean*
valueB:d
­
Cgradients/batch_normalization_2/moments/normalize/mean_grad/Shape_1Const*
dtype0*?
_class5
31loc:@batch_normalization_2/moments/normalize/mean*
valueB 
¼
Qgradients/batch_normalization_2/moments/normalize/mean_grad/BroadcastGradientArgsBroadcastGradientArgsAgradients/batch_normalization_2/moments/normalize/mean_grad/ShapeCgradients/batch_normalization_2/moments/normalize/mean_grad/Shape_1*?
_class5
31loc:@batch_normalization_2/moments/normalize/mean*
T0
Ô
?gradients/batch_normalization_2/moments/normalize/mean_grad/mulMulgradients/AddN_12/batch_normalization_2/moments/normalize/divisor*?
_class5
31loc:@batch_normalization_2/moments/normalize/mean*
T0
Á
?gradients/batch_normalization_2/moments/normalize/mean_grad/SumSum?gradients/batch_normalization_2/moments/normalize/mean_grad/mulQgradients/batch_normalization_2/moments/normalize/mean_grad/BroadcastGradientArgs*?
_class5
31loc:@batch_normalization_2/moments/normalize/mean*
T0*
	keep_dims( *

Tidx0
ª
Cgradients/batch_normalization_2/moments/normalize/mean_grad/ReshapeReshape?gradients/batch_normalization_2/moments/normalize/mean_grad/SumAgradients/batch_normalization_2/moments/normalize/mean_grad/Shape*?
_class5
31loc:@batch_normalization_2/moments/normalize/mean*
T0*
Tshape0
â
Agradients/batch_normalization_2/moments/normalize/mean_grad/mul_1Mul;batch_normalization_2/moments/sufficient_statistics/mean_ssgradients/AddN_12*?
_class5
31loc:@batch_normalization_2/moments/normalize/mean*
T0
Ç
Agradients/batch_normalization_2/moments/normalize/mean_grad/Sum_1SumAgradients/batch_normalization_2/moments/normalize/mean_grad/mul_1Sgradients/batch_normalization_2/moments/normalize/mean_grad/BroadcastGradientArgs:1*?
_class5
31loc:@batch_normalization_2/moments/normalize/mean*
T0*
	keep_dims( *

Tidx0
°
Egradients/batch_normalization_2/moments/normalize/mean_grad/Reshape_1ReshapeAgradients/batch_normalization_2/moments/normalize/mean_grad/Sum_1Cgradients/batch_normalization_2/moments/normalize/mean_grad/Shape_1*?
_class5
31loc:@batch_normalization_2/moments/normalize/mean*
T0*
Tshape0

Ogradients/batch_normalization_2/moments/sufficient_statistics/Square_grad/mul/xConstO^gradients/batch_normalization_2/moments/sufficient_statistics/var_ss_grad/Tile*
dtype0*M
_classC
A?loc:@batch_normalization_2/moments/sufficient_statistics/Square*
valueB
 *   @

Mgradients/batch_normalization_2/moments/sufficient_statistics/Square_grad/mulMulOgradients/batch_normalization_2/moments/sufficient_statistics/Square_grad/mul/xh1/Relu*M
_classC
A?loc:@batch_normalization_2/moments/sufficient_statistics/Square*
T0
Í
Ogradients/batch_normalization_2/moments/sufficient_statistics/Square_grad/mul_1MulNgradients/batch_normalization_2/moments/sufficient_statistics/var_ss_grad/TileMgradients/batch_normalization_2/moments/sufficient_statistics/Square_grad/mul*M
_classC
A?loc:@batch_normalization_2/moments/sufficient_statistics/Square*
T0
Ë
Pgradients/batch_normalization_2/moments/sufficient_statistics/mean_ss_grad/ShapeShapeh1/Relu*
out_type0*
T0*N
_classD
B@loc:@batch_normalization_2/moments/sufficient_statistics/mean_ss
É
Ogradients/batch_normalization_2/moments/sufficient_statistics/mean_ss_grad/SizeConst*
dtype0*N
_classD
B@loc:@batch_normalization_2/moments/sufficient_statistics/mean_ss*
value	B :
Î
Ngradients/batch_normalization_2/moments/sufficient_statistics/mean_ss_grad/addAddMbatch_normalization_2/moments/sufficient_statistics/mean_ss/reduction_indicesOgradients/batch_normalization_2/moments/sufficient_statistics/mean_ss_grad/Size*N
_classD
B@loc:@batch_normalization_2/moments/sufficient_statistics/mean_ss*
T0
Ï
Ngradients/batch_normalization_2/moments/sufficient_statistics/mean_ss_grad/modModNgradients/batch_normalization_2/moments/sufficient_statistics/mean_ss_grad/addOgradients/batch_normalization_2/moments/sufficient_statistics/mean_ss_grad/Size*N
_classD
B@loc:@batch_normalization_2/moments/sufficient_statistics/mean_ss*
T0
Ð
Rgradients/batch_normalization_2/moments/sufficient_statistics/mean_ss_grad/Shape_1Const*
dtype0*N
_classD
B@loc:@batch_normalization_2/moments/sufficient_statistics/mean_ss*
valueB:
Ð
Vgradients/batch_normalization_2/moments/sufficient_statistics/mean_ss_grad/range/startConst*
dtype0*N
_classD
B@loc:@batch_normalization_2/moments/sufficient_statistics/mean_ss*
value	B : 
Ð
Vgradients/batch_normalization_2/moments/sufficient_statistics/mean_ss_grad/range/deltaConst*
dtype0*N
_classD
B@loc:@batch_normalization_2/moments/sufficient_statistics/mean_ss*
value	B :
¶
Pgradients/batch_normalization_2/moments/sufficient_statistics/mean_ss_grad/rangeRangeVgradients/batch_normalization_2/moments/sufficient_statistics/mean_ss_grad/range/startOgradients/batch_normalization_2/moments/sufficient_statistics/mean_ss_grad/SizeVgradients/batch_normalization_2/moments/sufficient_statistics/mean_ss_grad/range/delta*N
_classD
B@loc:@batch_normalization_2/moments/sufficient_statistics/mean_ss*

Tidx0
Ï
Ugradients/batch_normalization_2/moments/sufficient_statistics/mean_ss_grad/Fill/valueConst*
dtype0*N
_classD
B@loc:@batch_normalization_2/moments/sufficient_statistics/mean_ss*
value	B :
Û
Ogradients/batch_normalization_2/moments/sufficient_statistics/mean_ss_grad/FillFillRgradients/batch_normalization_2/moments/sufficient_statistics/mean_ss_grad/Shape_1Ugradients/batch_normalization_2/moments/sufficient_statistics/mean_ss_grad/Fill/value*N
_classD
B@loc:@batch_normalization_2/moments/sufficient_statistics/mean_ss*
T0

Xgradients/batch_normalization_2/moments/sufficient_statistics/mean_ss_grad/DynamicStitchDynamicStitchPgradients/batch_normalization_2/moments/sufficient_statistics/mean_ss_grad/rangeNgradients/batch_normalization_2/moments/sufficient_statistics/mean_ss_grad/modPgradients/batch_normalization_2/moments/sufficient_statistics/mean_ss_grad/ShapeOgradients/batch_normalization_2/moments/sufficient_statistics/mean_ss_grad/Fill*N
_classD
B@loc:@batch_normalization_2/moments/sufficient_statistics/mean_ss*
T0*
N
Î
Tgradients/batch_normalization_2/moments/sufficient_statistics/mean_ss_grad/Maximum/yConst*
dtype0*N
_classD
B@loc:@batch_normalization_2/moments/sufficient_statistics/mean_ss*
value	B :
æ
Rgradients/batch_normalization_2/moments/sufficient_statistics/mean_ss_grad/MaximumMaximumXgradients/batch_normalization_2/moments/sufficient_statistics/mean_ss_grad/DynamicStitchTgradients/batch_normalization_2/moments/sufficient_statistics/mean_ss_grad/Maximum/y*N
_classD
B@loc:@batch_normalization_2/moments/sufficient_statistics/mean_ss*
T0
Ù
Sgradients/batch_normalization_2/moments/sufficient_statistics/mean_ss_grad/floordivDivPgradients/batch_normalization_2/moments/sufficient_statistics/mean_ss_grad/ShapeRgradients/batch_normalization_2/moments/sufficient_statistics/mean_ss_grad/Maximum*N
_classD
B@loc:@batch_normalization_2/moments/sufficient_statistics/mean_ss*
T0
ã
Rgradients/batch_normalization_2/moments/sufficient_statistics/mean_ss_grad/ReshapeReshapeCgradients/batch_normalization_2/moments/normalize/mean_grad/ReshapeXgradients/batch_normalization_2/moments/sufficient_statistics/mean_ss_grad/DynamicStitch*N
_classD
B@loc:@batch_normalization_2/moments/sufficient_statistics/mean_ss*
T0*
Tshape0
ë
Ogradients/batch_normalization_2/moments/sufficient_statistics/mean_ss_grad/TileTileRgradients/batch_normalization_2/moments/sufficient_statistics/mean_ss_grad/ReshapeSgradients/batch_normalization_2/moments/sufficient_statistics/mean_ss_grad/floordiv*

Tmultiples0*N
_classD
B@loc:@batch_normalization_2/moments/sufficient_statistics/mean_ss*
T0
ø
gradients/AddN_13AddNDgradients/batch_normalization_2/moments/normalize/Mul_grad/Reshape_1Egradients/batch_normalization_2/moments/normalize/mean_grad/Reshape_1*>
_class4
20loc:@batch_normalization_2/moments/normalize/Mul*
T0*
N
ð
Mgradients/batch_normalization_2/moments/normalize/divisor_grad/ReciprocalGradReciprocalGrad/batch_normalization_2/moments/normalize/divisorgradients/AddN_13*B
_class8
64loc:@batch_normalization_2/moments/normalize/divisor*
T0
ó
gradients/AddN_14AddNJgradients/batch_normalization_2/cond/batchnorm/mul_1/Switch_grad/cond_grad<gradients/batch_normalization_2/batchnorm/mul_1_grad/ReshapeOgradients/batch_normalization_2/moments/sufficient_statistics/Square_grad/mul_1Ogradients/batch_normalization_2/moments/sufficient_statistics/mean_ss_grad/Tile*
_class
loc:@h1/Relu*
T0*
N
l
gradients/h1/Relu_grad/ReluGradReluGradgradients/AddN_14h1/Relu*
_class
loc:@h1/Relu*
T0

%gradients/h1/BiasAdd_grad/BiasAddGradBiasAddGradgradients/h1/Relu_grad/ReluGrad*
_class
loc:@h1/BiasAdd*
T0*
data_formatNHWC
§
gradients/h1/MatMul_grad/MatMulMatMulgradients/h1/Relu_grad/ReluGradh1/kernel/read*
transpose_b(*
transpose_a( *
_class
loc:@h1/MatMul*
T0
»
!gradients/h1/MatMul_grad/MatMul_1MatMul batch_normalization_1/cond/Mergegradients/h1/Relu_grad/ReluGrad*
transpose_b( *
transpose_a(*
_class
loc:@h1/MatMul*
T0
¯
9gradients/batch_normalization_1/cond/Merge_grad/cond_gradSwitchgradients/h1/MatMul_grad/MatMul"batch_normalization_1/cond/pred_id*
_class
loc:@h1/MatMul*
T0
Ì
?gradients/batch_normalization_1/cond/batchnorm/add_1_grad/ShapeShape*batch_normalization_1/cond/batchnorm/mul_1*
out_type0*
T0*=
_class3
1/loc:@batch_normalization_1/cond/batchnorm/add_1
®
Agradients/batch_normalization_1/cond/batchnorm/add_1_grad/Shape_1Const*
dtype0*=
_class3
1/loc:@batch_normalization_1/cond/batchnorm/add_1*
valueB:
´
Ogradients/batch_normalization_1/cond/batchnorm/add_1_grad/BroadcastGradientArgsBroadcastGradientArgs?gradients/batch_normalization_1/cond/batchnorm/add_1_grad/ShapeAgradients/batch_normalization_1/cond/batchnorm/add_1_grad/Shape_1*=
_class3
1/loc:@batch_normalization_1/cond/batchnorm/add_1*
T0
µ
=gradients/batch_normalization_1/cond/batchnorm/add_1_grad/SumSum9gradients/batch_normalization_1/cond/Merge_grad/cond_gradOgradients/batch_normalization_1/cond/batchnorm/add_1_grad/BroadcastGradientArgs*=
_class3
1/loc:@batch_normalization_1/cond/batchnorm/add_1*
T0*
	keep_dims( *

Tidx0
¢
Agradients/batch_normalization_1/cond/batchnorm/add_1_grad/ReshapeReshape=gradients/batch_normalization_1/cond/batchnorm/add_1_grad/Sum?gradients/batch_normalization_1/cond/batchnorm/add_1_grad/Shape*=
_class3
1/loc:@batch_normalization_1/cond/batchnorm/add_1*
T0*
Tshape0
¹
?gradients/batch_normalization_1/cond/batchnorm/add_1_grad/Sum_1Sum9gradients/batch_normalization_1/cond/Merge_grad/cond_gradQgradients/batch_normalization_1/cond/batchnorm/add_1_grad/BroadcastGradientArgs:1*=
_class3
1/loc:@batch_normalization_1/cond/batchnorm/add_1*
T0*
	keep_dims( *

Tidx0
¨
Cgradients/batch_normalization_1/cond/batchnorm/add_1_grad/Reshape_1Reshape?gradients/batch_normalization_1/cond/batchnorm/add_1_grad/Sum_1Agradients/batch_normalization_1/cond/batchnorm/add_1_grad/Shape_1*=
_class3
1/loc:@batch_normalization_1/cond/batchnorm/add_1*
T0*
Tshape0
ª
gradients/Switch_8Switch%batch_normalization_1/batchnorm/add_1"batch_normalization_1/cond/pred_id*8
_class.
,*loc:@batch_normalization_1/batchnorm/add_1*
T0

gradients/Shape_9Shapegradients/Switch_8*
out_type0*
T0*8
_class.
,*loc:@batch_normalization_1/batchnorm/add_1
~
gradients/zeros_8/ConstConst*
dtype0*8
_class.
,*loc:@batch_normalization_1/batchnorm/add_1*
valueB
 *    

gradients/zeros_8Fillgradients/Shape_9gradients/zeros_8/Const*8
_class.
,*loc:@batch_normalization_1/batchnorm/add_1*
T0
á
<gradients/batch_normalization_1/cond/Switch_1_grad/cond_gradMerge;gradients/batch_normalization_1/cond/Merge_grad/cond_grad:1gradients/zeros_8*8
_class.
,*loc:@batch_normalization_1/batchnorm/add_1*
T0*
N
Ó
?gradients/batch_normalization_1/cond/batchnorm/mul_1_grad/ShapeShape1batch_normalization_1/cond/batchnorm/mul_1/Switch*
out_type0*
T0*=
_class3
1/loc:@batch_normalization_1/cond/batchnorm/mul_1
®
Agradients/batch_normalization_1/cond/batchnorm/mul_1_grad/Shape_1Const*
dtype0*=
_class3
1/loc:@batch_normalization_1/cond/batchnorm/mul_1*
valueB:
´
Ogradients/batch_normalization_1/cond/batchnorm/mul_1_grad/BroadcastGradientArgsBroadcastGradientArgs?gradients/batch_normalization_1/cond/batchnorm/mul_1_grad/ShapeAgradients/batch_normalization_1/cond/batchnorm/mul_1_grad/Shape_1*=
_class3
1/loc:@batch_normalization_1/cond/batchnorm/mul_1*
T0
ù
=gradients/batch_normalization_1/cond/batchnorm/mul_1_grad/mulMulAgradients/batch_normalization_1/cond/batchnorm/add_1_grad/Reshape(batch_normalization_1/cond/batchnorm/mul*=
_class3
1/loc:@batch_normalization_1/cond/batchnorm/mul_1*
T0
¹
=gradients/batch_normalization_1/cond/batchnorm/mul_1_grad/SumSum=gradients/batch_normalization_1/cond/batchnorm/mul_1_grad/mulOgradients/batch_normalization_1/cond/batchnorm/mul_1_grad/BroadcastGradientArgs*=
_class3
1/loc:@batch_normalization_1/cond/batchnorm/mul_1*
T0*
	keep_dims( *

Tidx0
¢
Agradients/batch_normalization_1/cond/batchnorm/mul_1_grad/ReshapeReshape=gradients/batch_normalization_1/cond/batchnorm/mul_1_grad/Sum?gradients/batch_normalization_1/cond/batchnorm/mul_1_grad/Shape*=
_class3
1/loc:@batch_normalization_1/cond/batchnorm/mul_1*
T0*
Tshape0

?gradients/batch_normalization_1/cond/batchnorm/mul_1_grad/mul_1Mul1batch_normalization_1/cond/batchnorm/mul_1/SwitchAgradients/batch_normalization_1/cond/batchnorm/add_1_grad/Reshape*=
_class3
1/loc:@batch_normalization_1/cond/batchnorm/mul_1*
T0
¿
?gradients/batch_normalization_1/cond/batchnorm/mul_1_grad/Sum_1Sum?gradients/batch_normalization_1/cond/batchnorm/mul_1_grad/mul_1Qgradients/batch_normalization_1/cond/batchnorm/mul_1_grad/BroadcastGradientArgs:1*=
_class3
1/loc:@batch_normalization_1/cond/batchnorm/mul_1*
T0*
	keep_dims( *

Tidx0
¨
Cgradients/batch_normalization_1/cond/batchnorm/mul_1_grad/Reshape_1Reshape?gradients/batch_normalization_1/cond/batchnorm/mul_1_grad/Sum_1Agradients/batch_normalization_1/cond/batchnorm/mul_1_grad/Shape_1*=
_class3
1/loc:@batch_normalization_1/cond/batchnorm/mul_1*
T0*
Tshape0
¨
=gradients/batch_normalization_1/cond/batchnorm/sub_grad/ShapeConst*
dtype0*;
_class1
/-loc:@batch_normalization_1/cond/batchnorm/sub*
valueB:
ª
?gradients/batch_normalization_1/cond/batchnorm/sub_grad/Shape_1Const*
dtype0*;
_class1
/-loc:@batch_normalization_1/cond/batchnorm/sub*
valueB:
¬
Mgradients/batch_normalization_1/cond/batchnorm/sub_grad/BroadcastGradientArgsBroadcastGradientArgs=gradients/batch_normalization_1/cond/batchnorm/sub_grad/Shape?gradients/batch_normalization_1/cond/batchnorm/sub_grad/Shape_1*;
_class1
/-loc:@batch_normalization_1/cond/batchnorm/sub*
T0
¹
;gradients/batch_normalization_1/cond/batchnorm/sub_grad/SumSumCgradients/batch_normalization_1/cond/batchnorm/add_1_grad/Reshape_1Mgradients/batch_normalization_1/cond/batchnorm/sub_grad/BroadcastGradientArgs*;
_class1
/-loc:@batch_normalization_1/cond/batchnorm/sub*
T0*
	keep_dims( *

Tidx0

?gradients/batch_normalization_1/cond/batchnorm/sub_grad/ReshapeReshape;gradients/batch_normalization_1/cond/batchnorm/sub_grad/Sum=gradients/batch_normalization_1/cond/batchnorm/sub_grad/Shape*;
_class1
/-loc:@batch_normalization_1/cond/batchnorm/sub*
T0*
Tshape0
½
=gradients/batch_normalization_1/cond/batchnorm/sub_grad/Sum_1SumCgradients/batch_normalization_1/cond/batchnorm/add_1_grad/Reshape_1Ogradients/batch_normalization_1/cond/batchnorm/sub_grad/BroadcastGradientArgs:1*;
_class1
/-loc:@batch_normalization_1/cond/batchnorm/sub*
T0*
	keep_dims( *

Tidx0
Ç
;gradients/batch_normalization_1/cond/batchnorm/sub_grad/NegNeg=gradients/batch_normalization_1/cond/batchnorm/sub_grad/Sum_1*;
_class1
/-loc:@batch_normalization_1/cond/batchnorm/sub*
T0

Agradients/batch_normalization_1/cond/batchnorm/sub_grad/Reshape_1Reshape;gradients/batch_normalization_1/cond/batchnorm/sub_grad/Neg?gradients/batch_normalization_1/cond/batchnorm/sub_grad/Shape_1*;
_class1
/-loc:@batch_normalization_1/cond/batchnorm/sub*
T0*
Tshape0
½
:gradients/batch_normalization_1/batchnorm/add_1_grad/ShapeShape%batch_normalization_1/batchnorm/mul_1*
out_type0*
T0*8
_class.
,*loc:@batch_normalization_1/batchnorm/add_1
¤
<gradients/batch_normalization_1/batchnorm/add_1_grad/Shape_1Const*
dtype0*8
_class.
,*loc:@batch_normalization_1/batchnorm/add_1*
valueB:
 
Jgradients/batch_normalization_1/batchnorm/add_1_grad/BroadcastGradientArgsBroadcastGradientArgs:gradients/batch_normalization_1/batchnorm/add_1_grad/Shape<gradients/batch_normalization_1/batchnorm/add_1_grad/Shape_1*8
_class.
,*loc:@batch_normalization_1/batchnorm/add_1*
T0
©
8gradients/batch_normalization_1/batchnorm/add_1_grad/SumSum<gradients/batch_normalization_1/cond/Switch_1_grad/cond_gradJgradients/batch_normalization_1/batchnorm/add_1_grad/BroadcastGradientArgs*8
_class.
,*loc:@batch_normalization_1/batchnorm/add_1*
T0*
	keep_dims( *

Tidx0

<gradients/batch_normalization_1/batchnorm/add_1_grad/ReshapeReshape8gradients/batch_normalization_1/batchnorm/add_1_grad/Sum:gradients/batch_normalization_1/batchnorm/add_1_grad/Shape*8
_class.
,*loc:@batch_normalization_1/batchnorm/add_1*
T0*
Tshape0
­
:gradients/batch_normalization_1/batchnorm/add_1_grad/Sum_1Sum<gradients/batch_normalization_1/cond/Switch_1_grad/cond_gradLgradients/batch_normalization_1/batchnorm/add_1_grad/BroadcastGradientArgs:1*8
_class.
,*loc:@batch_normalization_1/batchnorm/add_1*
T0*
	keep_dims( *

Tidx0

>gradients/batch_normalization_1/batchnorm/add_1_grad/Reshape_1Reshape:gradients/batch_normalization_1/batchnorm/add_1_grad/Sum_1<gradients/batch_normalization_1/batchnorm/add_1_grad/Shape_1*8
_class.
,*loc:@batch_normalization_1/batchnorm/add_1*
T0*
Tshape0

gradients/Switch_9Switchbatch_normalization_1/beta/read"batch_normalization_1/cond/pred_id*-
_class#
!loc:@batch_normalization_1/beta*
T0
y
gradients/Shape_10Shapegradients/Switch_9:1*
out_type0*
T0*-
_class#
!loc:@batch_normalization_1/beta
s
gradients/zeros_9/ConstConst*
dtype0*-
_class#
!loc:@batch_normalization_1/beta*
valueB
 *    
~
gradients/zeros_9Fillgradients/Shape_10gradients/zeros_9/Const*-
_class#
!loc:@batch_normalization_1/beta*
T0
æ
Hgradients/batch_normalization_1/cond/batchnorm/sub/Switch_grad/cond_gradMerge?gradients/batch_normalization_1/cond/batchnorm/sub_grad/Reshapegradients/zeros_9*-
_class#
!loc:@batch_normalization_1/beta*
T0*
N
¬
?gradients/batch_normalization_1/cond/batchnorm/mul_2_grad/ShapeConst*
dtype0*=
_class3
1/loc:@batch_normalization_1/cond/batchnorm/mul_2*
valueB:
®
Agradients/batch_normalization_1/cond/batchnorm/mul_2_grad/Shape_1Const*
dtype0*=
_class3
1/loc:@batch_normalization_1/cond/batchnorm/mul_2*
valueB:
´
Ogradients/batch_normalization_1/cond/batchnorm/mul_2_grad/BroadcastGradientArgsBroadcastGradientArgs?gradients/batch_normalization_1/cond/batchnorm/mul_2_grad/ShapeAgradients/batch_normalization_1/cond/batchnorm/mul_2_grad/Shape_1*=
_class3
1/loc:@batch_normalization_1/cond/batchnorm/mul_2*
T0
ù
=gradients/batch_normalization_1/cond/batchnorm/mul_2_grad/mulMulAgradients/batch_normalization_1/cond/batchnorm/sub_grad/Reshape_1(batch_normalization_1/cond/batchnorm/mul*=
_class3
1/loc:@batch_normalization_1/cond/batchnorm/mul_2*
T0
¹
=gradients/batch_normalization_1/cond/batchnorm/mul_2_grad/SumSum=gradients/batch_normalization_1/cond/batchnorm/mul_2_grad/mulOgradients/batch_normalization_1/cond/batchnorm/mul_2_grad/BroadcastGradientArgs*=
_class3
1/loc:@batch_normalization_1/cond/batchnorm/mul_2*
T0*
	keep_dims( *

Tidx0
¢
Agradients/batch_normalization_1/cond/batchnorm/mul_2_grad/ReshapeReshape=gradients/batch_normalization_1/cond/batchnorm/mul_2_grad/Sum?gradients/batch_normalization_1/cond/batchnorm/mul_2_grad/Shape*=
_class3
1/loc:@batch_normalization_1/cond/batchnorm/mul_2*
T0*
Tshape0

?gradients/batch_normalization_1/cond/batchnorm/mul_2_grad/mul_1Mul1batch_normalization_1/cond/batchnorm/mul_2/SwitchAgradients/batch_normalization_1/cond/batchnorm/sub_grad/Reshape_1*=
_class3
1/loc:@batch_normalization_1/cond/batchnorm/mul_2*
T0
¿
?gradients/batch_normalization_1/cond/batchnorm/mul_2_grad/Sum_1Sum?gradients/batch_normalization_1/cond/batchnorm/mul_2_grad/mul_1Qgradients/batch_normalization_1/cond/batchnorm/mul_2_grad/BroadcastGradientArgs:1*=
_class3
1/loc:@batch_normalization_1/cond/batchnorm/mul_2*
T0*
	keep_dims( *

Tidx0
¨
Cgradients/batch_normalization_1/cond/batchnorm/mul_2_grad/Reshape_1Reshape?gradients/batch_normalization_1/cond/batchnorm/mul_2_grad/Sum_1Agradients/batch_normalization_1/cond/batchnorm/mul_2_grad/Shape_1*=
_class3
1/loc:@batch_normalization_1/cond/batchnorm/mul_2*
T0*
Tshape0
§
:gradients/batch_normalization_1/batchnorm/mul_1_grad/ShapeShapes/strided_slice*
out_type0*
T0*8
_class.
,*loc:@batch_normalization_1/batchnorm/mul_1
¤
<gradients/batch_normalization_1/batchnorm/mul_1_grad/Shape_1Const*
dtype0*8
_class.
,*loc:@batch_normalization_1/batchnorm/mul_1*
valueB:
 
Jgradients/batch_normalization_1/batchnorm/mul_1_grad/BroadcastGradientArgsBroadcastGradientArgs:gradients/batch_normalization_1/batchnorm/mul_1_grad/Shape<gradients/batch_normalization_1/batchnorm/mul_1_grad/Shape_1*8
_class.
,*loc:@batch_normalization_1/batchnorm/mul_1*
T0
å
8gradients/batch_normalization_1/batchnorm/mul_1_grad/mulMul<gradients/batch_normalization_1/batchnorm/add_1_grad/Reshape#batch_normalization_1/batchnorm/mul*8
_class.
,*loc:@batch_normalization_1/batchnorm/mul_1*
T0
¥
8gradients/batch_normalization_1/batchnorm/mul_1_grad/SumSum8gradients/batch_normalization_1/batchnorm/mul_1_grad/mulJgradients/batch_normalization_1/batchnorm/mul_1_grad/BroadcastGradientArgs*8
_class.
,*loc:@batch_normalization_1/batchnorm/mul_1*
T0*
	keep_dims( *

Tidx0

<gradients/batch_normalization_1/batchnorm/mul_1_grad/ReshapeReshape8gradients/batch_normalization_1/batchnorm/mul_1_grad/Sum:gradients/batch_normalization_1/batchnorm/mul_1_grad/Shape*8
_class.
,*loc:@batch_normalization_1/batchnorm/mul_1*
T0*
Tshape0
Ó
:gradients/batch_normalization_1/batchnorm/mul_1_grad/mul_1Muls/strided_slice<gradients/batch_normalization_1/batchnorm/add_1_grad/Reshape*8
_class.
,*loc:@batch_normalization_1/batchnorm/mul_1*
T0
«
:gradients/batch_normalization_1/batchnorm/mul_1_grad/Sum_1Sum:gradients/batch_normalization_1/batchnorm/mul_1_grad/mul_1Lgradients/batch_normalization_1/batchnorm/mul_1_grad/BroadcastGradientArgs:1*8
_class.
,*loc:@batch_normalization_1/batchnorm/mul_1*
T0*
	keep_dims( *

Tidx0

>gradients/batch_normalization_1/batchnorm/mul_1_grad/Reshape_1Reshape:gradients/batch_normalization_1/batchnorm/mul_1_grad/Sum_1<gradients/batch_normalization_1/batchnorm/mul_1_grad/Shape_1*8
_class.
,*loc:@batch_normalization_1/batchnorm/mul_1*
T0*
Tshape0

8gradients/batch_normalization_1/batchnorm/sub_grad/ShapeConst*
dtype0*6
_class,
*(loc:@batch_normalization_1/batchnorm/sub*
valueB:
 
:gradients/batch_normalization_1/batchnorm/sub_grad/Shape_1Const*
dtype0*6
_class,
*(loc:@batch_normalization_1/batchnorm/sub*
valueB:

Hgradients/batch_normalization_1/batchnorm/sub_grad/BroadcastGradientArgsBroadcastGradientArgs8gradients/batch_normalization_1/batchnorm/sub_grad/Shape:gradients/batch_normalization_1/batchnorm/sub_grad/Shape_1*6
_class,
*(loc:@batch_normalization_1/batchnorm/sub*
T0
¥
6gradients/batch_normalization_1/batchnorm/sub_grad/SumSum>gradients/batch_normalization_1/batchnorm/add_1_grad/Reshape_1Hgradients/batch_normalization_1/batchnorm/sub_grad/BroadcastGradientArgs*6
_class,
*(loc:@batch_normalization_1/batchnorm/sub*
T0*
	keep_dims( *

Tidx0

:gradients/batch_normalization_1/batchnorm/sub_grad/ReshapeReshape6gradients/batch_normalization_1/batchnorm/sub_grad/Sum8gradients/batch_normalization_1/batchnorm/sub_grad/Shape*6
_class,
*(loc:@batch_normalization_1/batchnorm/sub*
T0*
Tshape0
©
8gradients/batch_normalization_1/batchnorm/sub_grad/Sum_1Sum>gradients/batch_normalization_1/batchnorm/add_1_grad/Reshape_1Jgradients/batch_normalization_1/batchnorm/sub_grad/BroadcastGradientArgs:1*6
_class,
*(loc:@batch_normalization_1/batchnorm/sub*
T0*
	keep_dims( *

Tidx0
¸
6gradients/batch_normalization_1/batchnorm/sub_grad/NegNeg8gradients/batch_normalization_1/batchnorm/sub_grad/Sum_1*6
_class,
*(loc:@batch_normalization_1/batchnorm/sub*
T0

<gradients/batch_normalization_1/batchnorm/sub_grad/Reshape_1Reshape6gradients/batch_normalization_1/batchnorm/sub_grad/Neg:gradients/batch_normalization_1/batchnorm/sub_grad/Shape_1*6
_class,
*(loc:@batch_normalization_1/batchnorm/sub*
T0*
Tshape0
ô
gradients/AddN_15AddNCgradients/batch_normalization_1/cond/batchnorm/mul_1_grad/Reshape_1Cgradients/batch_normalization_1/cond/batchnorm/mul_2_grad/Reshape_1*=
_class3
1/loc:@batch_normalization_1/cond/batchnorm/mul_1*
T0*
N
¨
=gradients/batch_normalization_1/cond/batchnorm/mul_grad/ShapeConst*
dtype0*;
_class1
/-loc:@batch_normalization_1/cond/batchnorm/mul*
valueB:
ª
?gradients/batch_normalization_1/cond/batchnorm/mul_grad/Shape_1Const*
dtype0*;
_class1
/-loc:@batch_normalization_1/cond/batchnorm/mul*
valueB:
¬
Mgradients/batch_normalization_1/cond/batchnorm/mul_grad/BroadcastGradientArgsBroadcastGradientArgs=gradients/batch_normalization_1/cond/batchnorm/mul_grad/Shape?gradients/batch_normalization_1/cond/batchnorm/mul_grad/Shape_1*;
_class1
/-loc:@batch_normalization_1/cond/batchnorm/mul*
T0
Ì
;gradients/batch_normalization_1/cond/batchnorm/mul_grad/mulMulgradients/AddN_15/batch_normalization_1/cond/batchnorm/mul/Switch*;
_class1
/-loc:@batch_normalization_1/cond/batchnorm/mul*
T0
±
;gradients/batch_normalization_1/cond/batchnorm/mul_grad/SumSum;gradients/batch_normalization_1/cond/batchnorm/mul_grad/mulMgradients/batch_normalization_1/cond/batchnorm/mul_grad/BroadcastGradientArgs*;
_class1
/-loc:@batch_normalization_1/cond/batchnorm/mul*
T0*
	keep_dims( *

Tidx0

?gradients/batch_normalization_1/cond/batchnorm/mul_grad/ReshapeReshape;gradients/batch_normalization_1/cond/batchnorm/mul_grad/Sum=gradients/batch_normalization_1/cond/batchnorm/mul_grad/Shape*;
_class1
/-loc:@batch_normalization_1/cond/batchnorm/mul*
T0*
Tshape0
É
=gradients/batch_normalization_1/cond/batchnorm/mul_grad/mul_1Mul*batch_normalization_1/cond/batchnorm/Rsqrtgradients/AddN_15*;
_class1
/-loc:@batch_normalization_1/cond/batchnorm/mul*
T0
·
=gradients/batch_normalization_1/cond/batchnorm/mul_grad/Sum_1Sum=gradients/batch_normalization_1/cond/batchnorm/mul_grad/mul_1Ogradients/batch_normalization_1/cond/batchnorm/mul_grad/BroadcastGradientArgs:1*;
_class1
/-loc:@batch_normalization_1/cond/batchnorm/mul*
T0*
	keep_dims( *

Tidx0
 
Agradients/batch_normalization_1/cond/batchnorm/mul_grad/Reshape_1Reshape=gradients/batch_normalization_1/cond/batchnorm/mul_grad/Sum_1?gradients/batch_normalization_1/cond/batchnorm/mul_grad/Shape_1*;
_class1
/-loc:@batch_normalization_1/cond/batchnorm/mul*
T0*
Tshape0
à
gradients/AddN_16AddNHgradients/batch_normalization_1/cond/batchnorm/sub/Switch_grad/cond_grad:gradients/batch_normalization_1/batchnorm/sub_grad/Reshape*-
_class#
!loc:@batch_normalization_1/beta*
T0*
N
¢
:gradients/batch_normalization_1/batchnorm/mul_2_grad/ShapeConst*
dtype0*8
_class.
,*loc:@batch_normalization_1/batchnorm/mul_2*
valueB:
¤
<gradients/batch_normalization_1/batchnorm/mul_2_grad/Shape_1Const*
dtype0*8
_class.
,*loc:@batch_normalization_1/batchnorm/mul_2*
valueB:
 
Jgradients/batch_normalization_1/batchnorm/mul_2_grad/BroadcastGradientArgsBroadcastGradientArgs:gradients/batch_normalization_1/batchnorm/mul_2_grad/Shape<gradients/batch_normalization_1/batchnorm/mul_2_grad/Shape_1*8
_class.
,*loc:@batch_normalization_1/batchnorm/mul_2*
T0
å
8gradients/batch_normalization_1/batchnorm/mul_2_grad/mulMul<gradients/batch_normalization_1/batchnorm/sub_grad/Reshape_1#batch_normalization_1/batchnorm/mul*8
_class.
,*loc:@batch_normalization_1/batchnorm/mul_2*
T0
¥
8gradients/batch_normalization_1/batchnorm/mul_2_grad/SumSum8gradients/batch_normalization_1/batchnorm/mul_2_grad/mulJgradients/batch_normalization_1/batchnorm/mul_2_grad/BroadcastGradientArgs*8
_class.
,*loc:@batch_normalization_1/batchnorm/mul_2*
T0*
	keep_dims( *

Tidx0

<gradients/batch_normalization_1/batchnorm/mul_2_grad/ReshapeReshape8gradients/batch_normalization_1/batchnorm/mul_2_grad/Sum:gradients/batch_normalization_1/batchnorm/mul_2_grad/Shape*8
_class.
,*loc:@batch_normalization_1/batchnorm/mul_2*
T0*
Tshape0
ð
:gradients/batch_normalization_1/batchnorm/mul_2_grad/mul_1Mul,batch_normalization_1/moments/normalize/mean<gradients/batch_normalization_1/batchnorm/sub_grad/Reshape_1*8
_class.
,*loc:@batch_normalization_1/batchnorm/mul_2*
T0
«
:gradients/batch_normalization_1/batchnorm/mul_2_grad/Sum_1Sum:gradients/batch_normalization_1/batchnorm/mul_2_grad/mul_1Lgradients/batch_normalization_1/batchnorm/mul_2_grad/BroadcastGradientArgs:1*8
_class.
,*loc:@batch_normalization_1/batchnorm/mul_2*
T0*
	keep_dims( *

Tidx0

>gradients/batch_normalization_1/batchnorm/mul_2_grad/Reshape_1Reshape:gradients/batch_normalization_1/batchnorm/mul_2_grad/Sum_1<gradients/batch_normalization_1/batchnorm/mul_2_grad/Shape_1*8
_class.
,*loc:@batch_normalization_1/batchnorm/mul_2*
T0*
Tshape0

gradients/Switch_10Switch batch_normalization_1/gamma/read"batch_normalization_1/cond/pred_id*.
_class$
" loc:@batch_normalization_1/gamma*
T0
{
gradients/Shape_11Shapegradients/Switch_10:1*
out_type0*
T0*.
_class$
" loc:@batch_normalization_1/gamma
u
gradients/zeros_10/ConstConst*
dtype0*.
_class$
" loc:@batch_normalization_1/gamma*
valueB
 *    

gradients/zeros_10Fillgradients/Shape_11gradients/zeros_10/Const*.
_class$
" loc:@batch_normalization_1/gamma*
T0
ê
Hgradients/batch_normalization_1/cond/batchnorm/mul/Switch_grad/cond_gradMergeAgradients/batch_normalization_1/cond/batchnorm/mul_grad/Reshape_1gradients/zeros_10*.
_class$
" loc:@batch_normalization_1/gamma*
T0*
N
å
gradients/AddN_17AddN>gradients/batch_normalization_1/batchnorm/mul_1_grad/Reshape_1>gradients/batch_normalization_1/batchnorm/mul_2_grad/Reshape_1*8
_class.
,*loc:@batch_normalization_1/batchnorm/mul_1*
T0*
N

8gradients/batch_normalization_1/batchnorm/mul_grad/ShapeConst*
dtype0*6
_class,
*(loc:@batch_normalization_1/batchnorm/mul*
valueB:
 
:gradients/batch_normalization_1/batchnorm/mul_grad/Shape_1Const*
dtype0*6
_class,
*(loc:@batch_normalization_1/batchnorm/mul*
valueB:

Hgradients/batch_normalization_1/batchnorm/mul_grad/BroadcastGradientArgsBroadcastGradientArgs8gradients/batch_normalization_1/batchnorm/mul_grad/Shape:gradients/batch_normalization_1/batchnorm/mul_grad/Shape_1*6
_class,
*(loc:@batch_normalization_1/batchnorm/mul*
T0
³
6gradients/batch_normalization_1/batchnorm/mul_grad/mulMulgradients/AddN_17 batch_normalization_1/gamma/read*6
_class,
*(loc:@batch_normalization_1/batchnorm/mul*
T0

6gradients/batch_normalization_1/batchnorm/mul_grad/SumSum6gradients/batch_normalization_1/batchnorm/mul_grad/mulHgradients/batch_normalization_1/batchnorm/mul_grad/BroadcastGradientArgs*6
_class,
*(loc:@batch_normalization_1/batchnorm/mul*
T0*
	keep_dims( *

Tidx0

:gradients/batch_normalization_1/batchnorm/mul_grad/ReshapeReshape6gradients/batch_normalization_1/batchnorm/mul_grad/Sum8gradients/batch_normalization_1/batchnorm/mul_grad/Shape*6
_class,
*(loc:@batch_normalization_1/batchnorm/mul*
T0*
Tshape0
º
8gradients/batch_normalization_1/batchnorm/mul_grad/mul_1Mul%batch_normalization_1/batchnorm/Rsqrtgradients/AddN_17*6
_class,
*(loc:@batch_normalization_1/batchnorm/mul*
T0
£
8gradients/batch_normalization_1/batchnorm/mul_grad/Sum_1Sum8gradients/batch_normalization_1/batchnorm/mul_grad/mul_1Jgradients/batch_normalization_1/batchnorm/mul_grad/BroadcastGradientArgs:1*6
_class,
*(loc:@batch_normalization_1/batchnorm/mul*
T0*
	keep_dims( *

Tidx0

<gradients/batch_normalization_1/batchnorm/mul_grad/Reshape_1Reshape8gradients/batch_normalization_1/batchnorm/mul_grad/Sum_1:gradients/batch_normalization_1/batchnorm/mul_grad/Shape_1*6
_class,
*(loc:@batch_normalization_1/batchnorm/mul*
T0*
Tshape0
ã
gradients/AddN_18AddNHgradients/batch_normalization_1/cond/batchnorm/mul/Switch_grad/cond_grad<gradients/batch_normalization_1/batchnorm/mul_grad/Reshape_1*.
_class$
" loc:@batch_normalization_1/gamma*
T0*
N
<
AssignAdd/valueConst*
dtype0*
valueB
 *  ?
n
	AssignAdd	AssignAdd
iterationsAssignAdd/value*
_class
loc:@iterations*
use_locking( *
T0
2
add/yConst*
dtype0*
valueB
 *  ?
+
addAdditerations/readadd/y*
T0
%
PowPowbeta_2/readadd*
T0
4
sub_1/xConst*
dtype0*
valueB
 *  ?
#
sub_1Subsub_1/xPow*
T0
4
Const_2Const*
dtype0*
valueB
 *    
4
Const_3Const*
dtype0*
valueB
 *  
9
clip_by_value/MinimumMinimumsub_1Const_3*
T0
A
clip_by_valueMaximumclip_by_value/MinimumConst_2*
T0
$
SqrtSqrtclip_by_value*
T0
'
Pow_1Powbeta_1/readadd*
T0
4
sub_2/xConst*
dtype0*
valueB
 *  ?
%
sub_2Subsub_2/xPow_1*
T0
"
div_1DivSqrtsub_2*
T0
%
mul_2Mullr/readdiv_1*
T0
8
Const_4Const*
dtype0*
valueB*    
V
VariableVariable*
dtype0*
shape:*
	container *
shared_name 
{
Variable/AssignAssignVariableConst_4*
validate_shape(*
_class
loc:@Variable*
use_locking(*
T0
I
Variable/readIdentityVariable*
_class
loc:@Variable*
T0
<
Const_5Const*
dtype0*
valueBd*    
\

Variable_1Variable*
dtype0*
shape
:d*
	container *
shared_name 

Variable_1/AssignAssign
Variable_1Const_5*
validate_shape(*
_class
loc:@Variable_1*
use_locking(*
T0
O
Variable_1/readIdentity
Variable_1*
_class
loc:@Variable_1*
T0
8
Const_6Const*
dtype0*
valueB*    
X

Variable_2Variable*
dtype0*
shape:*
	container *
shared_name 

Variable_2/AssignAssign
Variable_2Const_6*
validate_shape(*
_class
loc:@Variable_2*
use_locking(*
T0
O
Variable_2/readIdentity
Variable_2*
_class
loc:@Variable_2*
T0
8
Const_7Const*
dtype0*
valueB*    
X

Variable_3Variable*
dtype0*
shape:*
	container *
shared_name 

Variable_3/AssignAssign
Variable_3Const_7*
validate_shape(*
_class
loc:@Variable_3*
use_locking(*
T0
O
Variable_3/readIdentity
Variable_3*
_class
loc:@Variable_3*
T0
8
Const_8Const*
dtype0*
valueBd*    
X

Variable_4Variable*
dtype0*
shape:d*
	container *
shared_name 

Variable_4/AssignAssign
Variable_4Const_8*
validate_shape(*
_class
loc:@Variable_4*
use_locking(*
T0
O
Variable_4/readIdentity
Variable_4*
_class
loc:@Variable_4*
T0
8
Const_9Const*
dtype0*
valueBd*    
X

Variable_5Variable*
dtype0*
shape:d*
	container *
shared_name 

Variable_5/AssignAssign
Variable_5Const_9*
validate_shape(*
_class
loc:@Variable_5*
use_locking(*
T0
O
Variable_5/readIdentity
Variable_5*
_class
loc:@Variable_5*
T0
9
Const_10Const*
dtype0*
valueBd*    
X

Variable_6Variable*
dtype0*
shape:d*
	container *
shared_name 

Variable_6/AssignAssign
Variable_6Const_10*
validate_shape(*
_class
loc:@Variable_6*
use_locking(*
T0
O
Variable_6/readIdentity
Variable_6*
_class
loc:@Variable_6*
T0
9
Const_11Const*
dtype0*
valueBd*    
X

Variable_7Variable*
dtype0*
shape:d*
	container *
shared_name 

Variable_7/AssignAssign
Variable_7Const_11*
validate_shape(*
_class
loc:@Variable_7*
use_locking(*
T0
O
Variable_7/readIdentity
Variable_7*
_class
loc:@Variable_7*
T0
9
Const_12Const*
dtype0*
valueB*    
X

Variable_8Variable*
dtype0*
shape:*
	container *
shared_name 

Variable_8/AssignAssign
Variable_8Const_12*
validate_shape(*
_class
loc:@Variable_8*
use_locking(*
T0
O
Variable_8/readIdentity
Variable_8*
_class
loc:@Variable_8*
T0
=
Const_13Const*
dtype0*
valueBd*    
\

Variable_9Variable*
dtype0*
shape
:d*
	container *
shared_name 

Variable_9/AssignAssign
Variable_9Const_13*
validate_shape(*
_class
loc:@Variable_9*
use_locking(*
T0
O
Variable_9/readIdentity
Variable_9*
_class
loc:@Variable_9*
T0
9
Const_14Const*
dtype0*
valueB*    
Y
Variable_10Variable*
dtype0*
shape:*
	container *
shared_name 

Variable_10/AssignAssignVariable_10Const_14*
validate_shape(*
_class
loc:@Variable_10*
use_locking(*
T0
R
Variable_10/readIdentityVariable_10*
_class
loc:@Variable_10*
T0
=
Const_15Const*
dtype0*
valueBd*    
]
Variable_11Variable*
dtype0*
shape
:d*
	container *
shared_name 

Variable_11/AssignAssignVariable_11Const_15*
validate_shape(*
_class
loc:@Variable_11*
use_locking(*
T0
R
Variable_11/readIdentityVariable_11*
_class
loc:@Variable_11*
T0
9
Const_16Const*
dtype0*
valueBd*    
Y
Variable_12Variable*
dtype0*
shape:d*
	container *
shared_name 

Variable_12/AssignAssignVariable_12Const_16*
validate_shape(*
_class
loc:@Variable_12*
use_locking(*
T0
R
Variable_12/readIdentityVariable_12*
_class
loc:@Variable_12*
T0
=
Const_17Const*
dtype0*
valueBd*    
]
Variable_13Variable*
dtype0*
shape
:d*
	container *
shared_name 

Variable_13/AssignAssignVariable_13Const_17*
validate_shape(*
_class
loc:@Variable_13*
use_locking(*
T0
R
Variable_13/readIdentityVariable_13*
_class
loc:@Variable_13*
T0
9
Const_18Const*
dtype0*
valueBd*    
Y
Variable_14Variable*
dtype0*
shape:d*
	container *
shared_name 

Variable_14/AssignAssignVariable_14Const_18*
validate_shape(*
_class
loc:@Variable_14*
use_locking(*
T0
R
Variable_14/readIdentityVariable_14*
_class
loc:@Variable_14*
T0
=
Const_19Const*
dtype0*
valueBdd*    
]
Variable_15Variable*
dtype0*
shape
:dd*
	container *
shared_name 

Variable_15/AssignAssignVariable_15Const_19*
validate_shape(*
_class
loc:@Variable_15*
use_locking(*
T0
R
Variable_15/readIdentityVariable_15*
_class
loc:@Variable_15*
T0
9
Const_20Const*
dtype0*
valueB*    
Y
Variable_16Variable*
dtype0*
shape:*
	container *
shared_name 

Variable_16/AssignAssignVariable_16Const_20*
validate_shape(*
_class
loc:@Variable_16*
use_locking(*
T0
R
Variable_16/readIdentityVariable_16*
_class
loc:@Variable_16*
T0
=
Const_21Const*
dtype0*
valueBd*    
]
Variable_17Variable*
dtype0*
shape
:d*
	container *
shared_name 

Variable_17/AssignAssignVariable_17Const_21*
validate_shape(*
_class
loc:@Variable_17*
use_locking(*
T0
R
Variable_17/readIdentityVariable_17*
_class
loc:@Variable_17*
T0
9
Const_22Const*
dtype0*
valueB*    
Y
Variable_18Variable*
dtype0*
shape:*
	container *
shared_name 

Variable_18/AssignAssignVariable_18Const_22*
validate_shape(*
_class
loc:@Variable_18*
use_locking(*
T0
R
Variable_18/readIdentityVariable_18*
_class
loc:@Variable_18*
T0
9
Const_23Const*
dtype0*
valueB*    
Y
Variable_19Variable*
dtype0*
shape:*
	container *
shared_name 

Variable_19/AssignAssignVariable_19Const_23*
validate_shape(*
_class
loc:@Variable_19*
use_locking(*
T0
R
Variable_19/readIdentityVariable_19*
_class
loc:@Variable_19*
T0
9
Const_24Const*
dtype0*
valueBd*    
Y
Variable_20Variable*
dtype0*
shape:d*
	container *
shared_name 

Variable_20/AssignAssignVariable_20Const_24*
validate_shape(*
_class
loc:@Variable_20*
use_locking(*
T0
R
Variable_20/readIdentityVariable_20*
_class
loc:@Variable_20*
T0
9
Const_25Const*
dtype0*
valueBd*    
Y
Variable_21Variable*
dtype0*
shape:d*
	container *
shared_name 

Variable_21/AssignAssignVariable_21Const_25*
validate_shape(*
_class
loc:@Variable_21*
use_locking(*
T0
R
Variable_21/readIdentityVariable_21*
_class
loc:@Variable_21*
T0
9
Const_26Const*
dtype0*
valueBd*    
Y
Variable_22Variable*
dtype0*
shape:d*
	container *
shared_name 

Variable_22/AssignAssignVariable_22Const_26*
validate_shape(*
_class
loc:@Variable_22*
use_locking(*
T0
R
Variable_22/readIdentityVariable_22*
_class
loc:@Variable_22*
T0
9
Const_27Const*
dtype0*
valueBd*    
Y
Variable_23Variable*
dtype0*
shape:d*
	container *
shared_name 

Variable_23/AssignAssignVariable_23Const_27*
validate_shape(*
_class
loc:@Variable_23*
use_locking(*
T0
R
Variable_23/readIdentityVariable_23*
_class
loc:@Variable_23*
T0
9
Const_28Const*
dtype0*
valueB*    
Y
Variable_24Variable*
dtype0*
shape:*
	container *
shared_name 

Variable_24/AssignAssignVariable_24Const_28*
validate_shape(*
_class
loc:@Variable_24*
use_locking(*
T0
R
Variable_24/readIdentityVariable_24*
_class
loc:@Variable_24*
T0
=
Const_29Const*
dtype0*
valueBd*    
]
Variable_25Variable*
dtype0*
shape
:d*
	container *
shared_name 

Variable_25/AssignAssignVariable_25Const_29*
validate_shape(*
_class
loc:@Variable_25*
use_locking(*
T0
R
Variable_25/readIdentityVariable_25*
_class
loc:@Variable_25*
T0
9
Const_30Const*
dtype0*
valueB*    
Y
Variable_26Variable*
dtype0*
shape:*
	container *
shared_name 

Variable_26/AssignAssignVariable_26Const_30*
validate_shape(*
_class
loc:@Variable_26*
use_locking(*
T0
R
Variable_26/readIdentityVariable_26*
_class
loc:@Variable_26*
T0
=
Const_31Const*
dtype0*
valueBd*    
]
Variable_27Variable*
dtype0*
shape
:d*
	container *
shared_name 

Variable_27/AssignAssignVariable_27Const_31*
validate_shape(*
_class
loc:@Variable_27*
use_locking(*
T0
R
Variable_27/readIdentityVariable_27*
_class
loc:@Variable_27*
T0
9
Const_32Const*
dtype0*
valueBd*    
Y
Variable_28Variable*
dtype0*
shape:d*
	container *
shared_name 

Variable_28/AssignAssignVariable_28Const_32*
validate_shape(*
_class
loc:@Variable_28*
use_locking(*
T0
R
Variable_28/readIdentityVariable_28*
_class
loc:@Variable_28*
T0
=
Const_33Const*
dtype0*
valueBd*    
]
Variable_29Variable*
dtype0*
shape
:d*
	container *
shared_name 

Variable_29/AssignAssignVariable_29Const_33*
validate_shape(*
_class
loc:@Variable_29*
use_locking(*
T0
R
Variable_29/readIdentityVariable_29*
_class
loc:@Variable_29*
T0
9
Const_34Const*
dtype0*
valueBd*    
Y
Variable_30Variable*
dtype0*
shape:d*
	container *
shared_name 

Variable_30/AssignAssignVariable_30Const_34*
validate_shape(*
_class
loc:@Variable_30*
use_locking(*
T0
R
Variable_30/readIdentityVariable_30*
_class
loc:@Variable_30*
T0
=
Const_35Const*
dtype0*
valueBdd*    
]
Variable_31Variable*
dtype0*
shape
:dd*
	container *
shared_name 

Variable_31/AssignAssignVariable_31Const_35*
validate_shape(*
_class
loc:@Variable_31*
use_locking(*
T0
R
Variable_31/readIdentityVariable_31*
_class
loc:@Variable_31*
T0
1
mul_3Mulbeta_1/readVariable/read*
T0
4
sub_3/xConst*
dtype0*
valueB
 *  ?
+
sub_3Subsub_3/xbeta_1/read*
T0
B
mul_4Mulsub_3$gradients/V/BiasAdd_grad/BiasAddGrad*
T0
#
add_2Addmul_3mul_4*
T0
4
mul_5Mulbeta_2/readVariable_16/read*
T0
4
sub_4/xConst*
dtype0*
valueB
 *  ?
+
sub_4Subsub_4/xbeta_2/read*
T0
A
Square_1Square$gradients/V/BiasAdd_grad/BiasAddGrad*
T0
&
mul_6Mulsub_4Square_1*
T0
#
add_3Addmul_5mul_6*
T0
#
mul_7Mulmul_2add_2*
T0
5
Const_36Const*
dtype0*
valueB
 *    
5
Const_37Const*
dtype0*
valueB
 *  
<
clip_by_value_1/MinimumMinimumadd_3Const_37*
T0
F
clip_by_value_1Maximumclip_by_value_1/MinimumConst_36*
T0
(
Sqrt_1Sqrtclip_by_value_1*
T0
4
add_4/yConst*
dtype0*
valueB
 *wÌ+2
&
add_4AddSqrt_1add_4/y*
T0
#
div_2Divmul_7add_4*
T0
)
sub_5SubV/bias/readdiv_2*
T0
p
AssignAssignVariableadd_2*
validate_shape(*
_class
loc:@Variable*
use_locking(*
T0
x
Assign_1AssignVariable_16add_3*
validate_shape(*
_class
loc:@Variable_16*
use_locking(*
T0
n
Assign_2AssignV/biassub_5*
validate_shape(*
_class
loc:@V/bias*
use_locking(*
T0
3
mul_8Mulbeta_1/readVariable_1/read*
T0
4
sub_6/xConst*
dtype0*
valueB
 *  ?
+
sub_6Subsub_6/xbeta_1/read*
T0
>
mul_9Mulsub_6 gradients/V/MatMul_grad/MatMul_1*
T0
#
add_5Addmul_8mul_9*
T0
5
mul_10Mulbeta_2/readVariable_17/read*
T0
4
sub_7/xConst*
dtype0*
valueB
 *  ?
+
sub_7Subsub_7/xbeta_2/read*
T0
=
Square_2Square gradients/V/MatMul_grad/MatMul_1*
T0
'
mul_11Mulsub_7Square_2*
T0
%
add_6Addmul_10mul_11*
T0
$
mul_12Mulmul_2add_5*
T0
5
Const_38Const*
dtype0*
valueB
 *    
5
Const_39Const*
dtype0*
valueB
 *  
<
clip_by_value_2/MinimumMinimumadd_6Const_39*
T0
F
clip_by_value_2Maximumclip_by_value_2/MinimumConst_38*
T0
(
Sqrt_2Sqrtclip_by_value_2*
T0
4
add_7/yConst*
dtype0*
valueB
 *wÌ+2
&
add_7AddSqrt_2add_7/y*
T0
$
div_3Divmul_12add_7*
T0
+
sub_8SubV/kernel/readdiv_3*
T0
v
Assign_3Assign
Variable_1add_5*
validate_shape(*
_class
loc:@Variable_1*
use_locking(*
T0
x
Assign_4AssignVariable_17add_6*
validate_shape(*
_class
loc:@Variable_17*
use_locking(*
T0
r
Assign_5AssignV/kernelsub_8*
validate_shape(*
_class
loc:@V/kernel*
use_locking(*
T0
4
mul_13Mulbeta_1/readVariable_2/read*
T0
4
sub_9/xConst*
dtype0*
valueB
 *  ?
+
sub_9Subsub_9/xbeta_1/read*
T0
0
mul_14Mulsub_9gradients/AddN_16*
T0
%
add_8Addmul_13mul_14*
T0
5
mul_15Mulbeta_2/readVariable_18/read*
T0
5
sub_10/xConst*
dtype0*
valueB
 *  ?
-
sub_10Subsub_10/xbeta_2/read*
T0
.
Square_3Squaregradients/AddN_16*
T0
(
mul_16Mulsub_10Square_3*
T0
%
add_9Addmul_15mul_16*
T0
$
mul_17Mulmul_2add_8*
T0
5
Const_40Const*
dtype0*
valueB
 *    
5
Const_41Const*
dtype0*
valueB
 *  
<
clip_by_value_3/MinimumMinimumadd_9Const_41*
T0
F
clip_by_value_3Maximumclip_by_value_3/MinimumConst_40*
T0
(
Sqrt_3Sqrtclip_by_value_3*
T0
5
add_10/yConst*
dtype0*
valueB
 *wÌ+2
(
add_10AddSqrt_3add_10/y*
T0
%
div_4Divmul_17add_10*
T0
>
sub_11Subbatch_normalization_1/beta/readdiv_4*
T0
v
Assign_6Assign
Variable_2add_8*
validate_shape(*
_class
loc:@Variable_2*
use_locking(*
T0
x
Assign_7AssignVariable_18add_9*
validate_shape(*
_class
loc:@Variable_18*
use_locking(*
T0

Assign_8Assignbatch_normalization_1/betasub_11*
validate_shape(*-
_class#
!loc:@batch_normalization_1/beta*
use_locking(*
T0
4
mul_18Mulbeta_1/readVariable_3/read*
T0
5
sub_12/xConst*
dtype0*
valueB
 *  ?
-
sub_12Subsub_12/xbeta_1/read*
T0
1
mul_19Mulsub_12gradients/AddN_18*
T0
&
add_11Addmul_18mul_19*
T0
5
mul_20Mulbeta_2/readVariable_19/read*
T0
5
sub_13/xConst*
dtype0*
valueB
 *  ?
-
sub_13Subsub_13/xbeta_2/read*
T0
.
Square_4Squaregradients/AddN_18*
T0
(
mul_21Mulsub_13Square_4*
T0
&
add_12Addmul_20mul_21*
T0
%
mul_22Mulmul_2add_11*
T0
5
Const_42Const*
dtype0*
valueB
 *    
5
Const_43Const*
dtype0*
valueB
 *  
=
clip_by_value_4/MinimumMinimumadd_12Const_43*
T0
F
clip_by_value_4Maximumclip_by_value_4/MinimumConst_42*
T0
(
Sqrt_4Sqrtclip_by_value_4*
T0
5
add_13/yConst*
dtype0*
valueB
 *wÌ+2
(
add_13AddSqrt_4add_13/y*
T0
%
div_5Divmul_22add_13*
T0
?
sub_14Sub batch_normalization_1/gamma/readdiv_5*
T0
w
Assign_9Assign
Variable_3add_11*
validate_shape(*
_class
loc:@Variable_3*
use_locking(*
T0
z
	Assign_10AssignVariable_19add_12*
validate_shape(*
_class
loc:@Variable_19*
use_locking(*
T0

	Assign_11Assignbatch_normalization_1/gammasub_14*
validate_shape(*.
_class$
" loc:@batch_normalization_1/gamma*
use_locking(*
T0
4
mul_23Mulbeta_1/readVariable_4/read*
T0
5
sub_15/xConst*
dtype0*
valueB
 *  ?
-
sub_15Subsub_15/xbeta_1/read*
T0
0
mul_24Mulsub_15gradients/AddN_9*
T0
&
add_14Addmul_23mul_24*
T0
5
mul_25Mulbeta_2/readVariable_20/read*
T0
5
sub_16/xConst*
dtype0*
valueB
 *  ?
-
sub_16Subsub_16/xbeta_2/read*
T0
-
Square_5Squaregradients/AddN_9*
T0
(
mul_26Mulsub_16Square_5*
T0
&
add_15Addmul_25mul_26*
T0
%
mul_27Mulmul_2add_14*
T0
5
Const_44Const*
dtype0*
valueB
 *    
5
Const_45Const*
dtype0*
valueB
 *  
=
clip_by_value_5/MinimumMinimumadd_15Const_45*
T0
F
clip_by_value_5Maximumclip_by_value_5/MinimumConst_44*
T0
(
Sqrt_5Sqrtclip_by_value_5*
T0
5
add_16/yConst*
dtype0*
valueB
 *wÌ+2
(
add_16AddSqrt_5add_16/y*
T0
%
div_6Divmul_27add_16*
T0
>
sub_17Subbatch_normalization_2/beta/readdiv_6*
T0
x
	Assign_12Assign
Variable_4add_14*
validate_shape(*
_class
loc:@Variable_4*
use_locking(*
T0
z
	Assign_13AssignVariable_20add_15*
validate_shape(*
_class
loc:@Variable_20*
use_locking(*
T0

	Assign_14Assignbatch_normalization_2/betasub_17*
validate_shape(*-
_class#
!loc:@batch_normalization_2/beta*
use_locking(*
T0
4
mul_28Mulbeta_1/readVariable_5/read*
T0
5
sub_18/xConst*
dtype0*
valueB
 *  ?
-
sub_18Subsub_18/xbeta_1/read*
T0
1
mul_29Mulsub_18gradients/AddN_11*
T0
&
add_17Addmul_28mul_29*
T0
5
mul_30Mulbeta_2/readVariable_21/read*
T0
5
sub_19/xConst*
dtype0*
valueB
 *  ?
-
sub_19Subsub_19/xbeta_2/read*
T0
.
Square_6Squaregradients/AddN_11*
T0
(
mul_31Mulsub_19Square_6*
T0
&
add_18Addmul_30mul_31*
T0
%
mul_32Mulmul_2add_17*
T0
5
Const_46Const*
dtype0*
valueB
 *    
5
Const_47Const*
dtype0*
valueB
 *  
=
clip_by_value_6/MinimumMinimumadd_18Const_47*
T0
F
clip_by_value_6Maximumclip_by_value_6/MinimumConst_46*
T0
(
Sqrt_6Sqrtclip_by_value_6*
T0
5
add_19/yConst*
dtype0*
valueB
 *wÌ+2
(
add_19AddSqrt_6add_19/y*
T0
%
div_7Divmul_32add_19*
T0
?
sub_20Sub batch_normalization_2/gamma/readdiv_7*
T0
x
	Assign_15Assign
Variable_5add_17*
validate_shape(*
_class
loc:@Variable_5*
use_locking(*
T0
z
	Assign_16AssignVariable_21add_18*
validate_shape(*
_class
loc:@Variable_21*
use_locking(*
T0

	Assign_17Assignbatch_normalization_2/gammasub_20*
validate_shape(*.
_class$
" loc:@batch_normalization_2/gamma*
use_locking(*
T0
4
mul_33Mulbeta_1/readVariable_6/read*
T0
5
sub_21/xConst*
dtype0*
valueB
 *  ?
-
sub_21Subsub_21/xbeta_1/read*
T0
0
mul_34Mulsub_21gradients/AddN_2*
T0
&
add_20Addmul_33mul_34*
T0
5
mul_35Mulbeta_2/readVariable_22/read*
T0
5
sub_22/xConst*
dtype0*
valueB
 *  ?
-
sub_22Subsub_22/xbeta_2/read*
T0
-
Square_7Squaregradients/AddN_2*
T0
(
mul_36Mulsub_22Square_7*
T0
&
add_21Addmul_35mul_36*
T0
%
mul_37Mulmul_2add_20*
T0
5
Const_48Const*
dtype0*
valueB
 *    
5
Const_49Const*
dtype0*
valueB
 *  
=
clip_by_value_7/MinimumMinimumadd_21Const_49*
T0
F
clip_by_value_7Maximumclip_by_value_7/MinimumConst_48*
T0
(
Sqrt_7Sqrtclip_by_value_7*
T0
5
add_22/yConst*
dtype0*
valueB
 *wÌ+2
(
add_22AddSqrt_7add_22/y*
T0
%
div_8Divmul_37add_22*
T0
>
sub_23Subbatch_normalization_3/beta/readdiv_8*
T0
x
	Assign_18Assign
Variable_6add_20*
validate_shape(*
_class
loc:@Variable_6*
use_locking(*
T0
z
	Assign_19AssignVariable_22add_21*
validate_shape(*
_class
loc:@Variable_22*
use_locking(*
T0

	Assign_20Assignbatch_normalization_3/betasub_23*
validate_shape(*-
_class#
!loc:@batch_normalization_3/beta*
use_locking(*
T0
4
mul_38Mulbeta_1/readVariable_7/read*
T0
5
sub_24/xConst*
dtype0*
valueB
 *  ?
-
sub_24Subsub_24/xbeta_1/read*
T0
0
mul_39Mulsub_24gradients/AddN_4*
T0
&
add_23Addmul_38mul_39*
T0
5
mul_40Mulbeta_2/readVariable_23/read*
T0
5
sub_25/xConst*
dtype0*
valueB
 *  ?
-
sub_25Subsub_25/xbeta_2/read*
T0
-
Square_8Squaregradients/AddN_4*
T0
(
mul_41Mulsub_25Square_8*
T0
&
add_24Addmul_40mul_41*
T0
%
mul_42Mulmul_2add_23*
T0
5
Const_50Const*
dtype0*
valueB
 *    
5
Const_51Const*
dtype0*
valueB
 *  
=
clip_by_value_8/MinimumMinimumadd_24Const_51*
T0
F
clip_by_value_8Maximumclip_by_value_8/MinimumConst_50*
T0
(
Sqrt_8Sqrtclip_by_value_8*
T0
5
add_25/yConst*
dtype0*
valueB
 *wÌ+2
(
add_25AddSqrt_8add_25/y*
T0
%
div_9Divmul_42add_25*
T0
?
sub_26Sub batch_normalization_3/gamma/readdiv_9*
T0
x
	Assign_21Assign
Variable_7add_23*
validate_shape(*
_class
loc:@Variable_7*
use_locking(*
T0
z
	Assign_22AssignVariable_23add_24*
validate_shape(*
_class
loc:@Variable_23*
use_locking(*
T0

	Assign_23Assignbatch_normalization_3/gammasub_26*
validate_shape(*.
_class$
" loc:@batch_normalization_3/gamma*
use_locking(*
T0
4
mul_43Mulbeta_1/readVariable_8/read*
T0
5
sub_27/xConst*
dtype0*
valueB
 *  ?
-
sub_27Subsub_27/xbeta_1/read*
T0
J
mul_44Mulsub_27*gradients/dense_1/BiasAdd_grad/BiasAddGrad*
T0
&
add_26Addmul_43mul_44*
T0
5
mul_45Mulbeta_2/readVariable_24/read*
T0
5
sub_28/xConst*
dtype0*
valueB
 *  ?
-
sub_28Subsub_28/xbeta_2/read*
T0
G
Square_9Square*gradients/dense_1/BiasAdd_grad/BiasAddGrad*
T0
(
mul_46Mulsub_28Square_9*
T0
&
add_27Addmul_45mul_46*
T0
%
mul_47Mulmul_2add_26*
T0
5
Const_52Const*
dtype0*
valueB
 *    
5
Const_53Const*
dtype0*
valueB
 *  
=
clip_by_value_9/MinimumMinimumadd_27Const_53*
T0
F
clip_by_value_9Maximumclip_by_value_9/MinimumConst_52*
T0
(
Sqrt_9Sqrtclip_by_value_9*
T0
5
add_28/yConst*
dtype0*
valueB
 *wÌ+2
(
add_28AddSqrt_9add_28/y*
T0
&
div_10Divmul_47add_28*
T0
1
sub_29Subdense_1/bias/readdiv_10*
T0
x
	Assign_24Assign
Variable_8add_26*
validate_shape(*
_class
loc:@Variable_8*
use_locking(*
T0
z
	Assign_25AssignVariable_24add_27*
validate_shape(*
_class
loc:@Variable_24*
use_locking(*
T0
|
	Assign_26Assigndense_1/biassub_29*
validate_shape(*
_class
loc:@dense_1/bias*
use_locking(*
T0
4
mul_48Mulbeta_1/readVariable_9/read*
T0
5
sub_30/xConst*
dtype0*
valueB
 *  ?
-
sub_30Subsub_30/xbeta_1/read*
T0
F
mul_49Mulsub_30&gradients/dense_1/MatMul_grad/MatMul_1*
T0
&
add_29Addmul_48mul_49*
T0
5
mul_50Mulbeta_2/readVariable_25/read*
T0
5
sub_31/xConst*
dtype0*
valueB
 *  ?
-
sub_31Subsub_31/xbeta_2/read*
T0
D
	Square_10Square&gradients/dense_1/MatMul_grad/MatMul_1*
T0
)
mul_51Mulsub_31	Square_10*
T0
&
add_30Addmul_50mul_51*
T0
%
mul_52Mulmul_2add_29*
T0
5
Const_54Const*
dtype0*
valueB
 *    
5
Const_55Const*
dtype0*
valueB
 *  
>
clip_by_value_10/MinimumMinimumadd_30Const_55*
T0
H
clip_by_value_10Maximumclip_by_value_10/MinimumConst_54*
T0
*
Sqrt_10Sqrtclip_by_value_10*
T0
5
add_31/yConst*
dtype0*
valueB
 *wÌ+2
)
add_31AddSqrt_10add_31/y*
T0
&
div_11Divmul_52add_31*
T0
3
sub_32Subdense_1/kernel/readdiv_11*
T0
x
	Assign_27Assign
Variable_9add_29*
validate_shape(*
_class
loc:@Variable_9*
use_locking(*
T0
z
	Assign_28AssignVariable_25add_30*
validate_shape(*
_class
loc:@Variable_25*
use_locking(*
T0

	Assign_29Assigndense_1/kernelsub_32*
validate_shape(*!
_class
loc:@dense_1/kernel*
use_locking(*
T0
5
mul_53Mulbeta_1/readVariable_10/read*
T0
5
sub_33/xConst*
dtype0*
valueB
 *  ?
-
sub_33Subsub_33/xbeta_1/read*
T0
J
mul_54Mulsub_33*gradients/dense_2/BiasAdd_grad/BiasAddGrad*
T0
&
add_32Addmul_53mul_54*
T0
5
mul_55Mulbeta_2/readVariable_26/read*
T0
5
sub_34/xConst*
dtype0*
valueB
 *  ?
-
sub_34Subsub_34/xbeta_2/read*
T0
H
	Square_11Square*gradients/dense_2/BiasAdd_grad/BiasAddGrad*
T0
)
mul_56Mulsub_34	Square_11*
T0
&
add_33Addmul_55mul_56*
T0
%
mul_57Mulmul_2add_32*
T0
5
Const_56Const*
dtype0*
valueB
 *    
5
Const_57Const*
dtype0*
valueB
 *  
>
clip_by_value_11/MinimumMinimumadd_33Const_57*
T0
H
clip_by_value_11Maximumclip_by_value_11/MinimumConst_56*
T0
*
Sqrt_11Sqrtclip_by_value_11*
T0
5
add_34/yConst*
dtype0*
valueB
 *wÌ+2
)
add_34AddSqrt_11add_34/y*
T0
&
div_12Divmul_57add_34*
T0
1
sub_35Subdense_2/bias/readdiv_12*
T0
z
	Assign_30AssignVariable_10add_32*
validate_shape(*
_class
loc:@Variable_10*
use_locking(*
T0
z
	Assign_31AssignVariable_26add_33*
validate_shape(*
_class
loc:@Variable_26*
use_locking(*
T0
|
	Assign_32Assigndense_2/biassub_35*
validate_shape(*
_class
loc:@dense_2/bias*
use_locking(*
T0
5
mul_58Mulbeta_1/readVariable_11/read*
T0
5
sub_36/xConst*
dtype0*
valueB
 *  ?
-
sub_36Subsub_36/xbeta_1/read*
T0
F
mul_59Mulsub_36&gradients/dense_2/MatMul_grad/MatMul_1*
T0
&
add_35Addmul_58mul_59*
T0
5
mul_60Mulbeta_2/readVariable_27/read*
T0
5
sub_37/xConst*
dtype0*
valueB
 *  ?
-
sub_37Subsub_37/xbeta_2/read*
T0
D
	Square_12Square&gradients/dense_2/MatMul_grad/MatMul_1*
T0
)
mul_61Mulsub_37	Square_12*
T0
&
add_36Addmul_60mul_61*
T0
%
mul_62Mulmul_2add_35*
T0
5
Const_58Const*
dtype0*
valueB
 *    
5
Const_59Const*
dtype0*
valueB
 *  
>
clip_by_value_12/MinimumMinimumadd_36Const_59*
T0
H
clip_by_value_12Maximumclip_by_value_12/MinimumConst_58*
T0
*
Sqrt_12Sqrtclip_by_value_12*
T0
5
add_37/yConst*
dtype0*
valueB
 *wÌ+2
)
add_37AddSqrt_12add_37/y*
T0
&
div_13Divmul_62add_37*
T0
3
sub_38Subdense_2/kernel/readdiv_13*
T0
z
	Assign_33AssignVariable_11add_35*
validate_shape(*
_class
loc:@Variable_11*
use_locking(*
T0
z
	Assign_34AssignVariable_27add_36*
validate_shape(*
_class
loc:@Variable_27*
use_locking(*
T0

	Assign_35Assigndense_2/kernelsub_38*
validate_shape(*!
_class
loc:@dense_2/kernel*
use_locking(*
T0
5
mul_63Mulbeta_1/readVariable_12/read*
T0
5
sub_39/xConst*
dtype0*
valueB
 *  ?
-
sub_39Subsub_39/xbeta_1/read*
T0
E
mul_64Mulsub_39%gradients/h1/BiasAdd_grad/BiasAddGrad*
T0
&
add_38Addmul_63mul_64*
T0
5
mul_65Mulbeta_2/readVariable_28/read*
T0
5
sub_40/xConst*
dtype0*
valueB
 *  ?
-
sub_40Subsub_40/xbeta_2/read*
T0
C
	Square_13Square%gradients/h1/BiasAdd_grad/BiasAddGrad*
T0
)
mul_66Mulsub_40	Square_13*
T0
&
add_39Addmul_65mul_66*
T0
%
mul_67Mulmul_2add_38*
T0
5
Const_60Const*
dtype0*
valueB
 *    
5
Const_61Const*
dtype0*
valueB
 *  
>
clip_by_value_13/MinimumMinimumadd_39Const_61*
T0
H
clip_by_value_13Maximumclip_by_value_13/MinimumConst_60*
T0
*
Sqrt_13Sqrtclip_by_value_13*
T0
5
add_40/yConst*
dtype0*
valueB
 *wÌ+2
)
add_40AddSqrt_13add_40/y*
T0
&
div_14Divmul_67add_40*
T0
,
sub_41Subh1/bias/readdiv_14*
T0
z
	Assign_36AssignVariable_12add_38*
validate_shape(*
_class
loc:@Variable_12*
use_locking(*
T0
z
	Assign_37AssignVariable_28add_39*
validate_shape(*
_class
loc:@Variable_28*
use_locking(*
T0
r
	Assign_38Assignh1/biassub_41*
validate_shape(*
_class
loc:@h1/bias*
use_locking(*
T0
5
mul_68Mulbeta_1/readVariable_13/read*
T0
5
sub_42/xConst*
dtype0*
valueB
 *  ?
-
sub_42Subsub_42/xbeta_1/read*
T0
A
mul_69Mulsub_42!gradients/h1/MatMul_grad/MatMul_1*
T0
&
add_41Addmul_68mul_69*
T0
5
mul_70Mulbeta_2/readVariable_29/read*
T0
5
sub_43/xConst*
dtype0*
valueB
 *  ?
-
sub_43Subsub_43/xbeta_2/read*
T0
?
	Square_14Square!gradients/h1/MatMul_grad/MatMul_1*
T0
)
mul_71Mulsub_43	Square_14*
T0
&
add_42Addmul_70mul_71*
T0
%
mul_72Mulmul_2add_41*
T0
5
Const_62Const*
dtype0*
valueB
 *    
5
Const_63Const*
dtype0*
valueB
 *  
>
clip_by_value_14/MinimumMinimumadd_42Const_63*
T0
H
clip_by_value_14Maximumclip_by_value_14/MinimumConst_62*
T0
*
Sqrt_14Sqrtclip_by_value_14*
T0
5
add_43/yConst*
dtype0*
valueB
 *wÌ+2
)
add_43AddSqrt_14add_43/y*
T0
&
div_15Divmul_72add_43*
T0
.
sub_44Subh1/kernel/readdiv_15*
T0
z
	Assign_39AssignVariable_13add_41*
validate_shape(*
_class
loc:@Variable_13*
use_locking(*
T0
z
	Assign_40AssignVariable_29add_42*
validate_shape(*
_class
loc:@Variable_29*
use_locking(*
T0
v
	Assign_41Assign	h1/kernelsub_44*
validate_shape(*
_class
loc:@h1/kernel*
use_locking(*
T0
5
mul_73Mulbeta_1/readVariable_14/read*
T0
5
sub_45/xConst*
dtype0*
valueB
 *  ?
-
sub_45Subsub_45/xbeta_1/read*
T0
E
mul_74Mulsub_45%gradients/h2/BiasAdd_grad/BiasAddGrad*
T0
&
add_44Addmul_73mul_74*
T0
5
mul_75Mulbeta_2/readVariable_30/read*
T0
5
sub_46/xConst*
dtype0*
valueB
 *  ?
-
sub_46Subsub_46/xbeta_2/read*
T0
C
	Square_15Square%gradients/h2/BiasAdd_grad/BiasAddGrad*
T0
)
mul_76Mulsub_46	Square_15*
T0
&
add_45Addmul_75mul_76*
T0
%
mul_77Mulmul_2add_44*
T0
5
Const_64Const*
dtype0*
valueB
 *    
5
Const_65Const*
dtype0*
valueB
 *  
>
clip_by_value_15/MinimumMinimumadd_45Const_65*
T0
H
clip_by_value_15Maximumclip_by_value_15/MinimumConst_64*
T0
*
Sqrt_15Sqrtclip_by_value_15*
T0
5
add_46/yConst*
dtype0*
valueB
 *wÌ+2
)
add_46AddSqrt_15add_46/y*
T0
&
div_16Divmul_77add_46*
T0
,
sub_47Subh2/bias/readdiv_16*
T0
z
	Assign_42AssignVariable_14add_44*
validate_shape(*
_class
loc:@Variable_14*
use_locking(*
T0
z
	Assign_43AssignVariable_30add_45*
validate_shape(*
_class
loc:@Variable_30*
use_locking(*
T0
r
	Assign_44Assignh2/biassub_47*
validate_shape(*
_class
loc:@h2/bias*
use_locking(*
T0
5
mul_78Mulbeta_1/readVariable_15/read*
T0
5
sub_48/xConst*
dtype0*
valueB
 *  ?
-
sub_48Subsub_48/xbeta_1/read*
T0
A
mul_79Mulsub_48!gradients/h2/MatMul_grad/MatMul_1*
T0
&
add_47Addmul_78mul_79*
T0
5
mul_80Mulbeta_2/readVariable_31/read*
T0
5
sub_49/xConst*
dtype0*
valueB
 *  ?
-
sub_49Subsub_49/xbeta_2/read*
T0
?
	Square_16Square!gradients/h2/MatMul_grad/MatMul_1*
T0
)
mul_81Mulsub_49	Square_16*
T0
&
add_48Addmul_80mul_81*
T0
%
mul_82Mulmul_2add_47*
T0
5
Const_66Const*
dtype0*
valueB
 *    
5
Const_67Const*
dtype0*
valueB
 *  
>
clip_by_value_16/MinimumMinimumadd_48Const_67*
T0
H
clip_by_value_16Maximumclip_by_value_16/MinimumConst_66*
T0
*
Sqrt_16Sqrtclip_by_value_16*
T0
5
add_49/yConst*
dtype0*
valueB
 *wÌ+2
)
add_49AddSqrt_16add_49/y*
T0
&
div_17Divmul_82add_49*
T0
.
sub_50Subh2/kernel/readdiv_17*
T0
z
	Assign_45AssignVariable_15add_47*
validate_shape(*
_class
loc:@Variable_15*
use_locking(*
T0
z
	Assign_46AssignVariable_31add_48*
validate_shape(*
_class
loc:@Variable_31*
use_locking(*
T0
v
	Assign_47Assign	h2/kernelsub_50*
validate_shape(*
_class
loc:@h2/kernel*
use_locking(*
T0
Ð

group_depsNoOp^mul_1&^batch_normalization_1/AssignMovingAvg(^batch_normalization_1/AssignMovingAvg_1&^batch_normalization_2/AssignMovingAvg(^batch_normalization_2/AssignMovingAvg_1&^batch_normalization_3/AssignMovingAvg(^batch_normalization_3/AssignMovingAvg_1
^AssignAdd^Assign	^Assign_1	^Assign_2	^Assign_3	^Assign_4	^Assign_5	^Assign_6	^Assign_7	^Assign_8	^Assign_9
^Assign_10
^Assign_11
^Assign_12
^Assign_13
^Assign_14
^Assign_15
^Assign_16
^Assign_17
^Assign_18
^Assign_19
^Assign_20
^Assign_21
^Assign_22
^Assign_23
^Assign_24
^Assign_25
^Assign_26
^Assign_27
^Assign_28
^Assign_29
^Assign_30
^Assign_31
^Assign_32
^Assign_33
^Assign_34
^Assign_35
^Assign_36
^Assign_37
^Assign_38
^Assign_39
^Assign_40
^Assign_41
^Assign_42
^Assign_43
^Assign_44
^Assign_45
^Assign_46
^Assign_47

group_deps_1NoOp^Q/add

initNoOp#^batch_normalization_1/gamma/Assign"^batch_normalization_1/beta/Assign)^batch_normalization_1/moving_mean/Assign-^batch_normalization_1/moving_variance/Assign^h1/kernel/Assign^h1/bias/Assign#^batch_normalization_2/gamma/Assign"^batch_normalization_2/beta/Assign)^batch_normalization_2/moving_mean/Assign-^batch_normalization_2/moving_variance/Assign^h2/kernel/Assign^h2/bias/Assign#^batch_normalization_3/gamma/Assign"^batch_normalization_3/beta/Assign)^batch_normalization_3/moving_mean/Assign-^batch_normalization_3/moving_variance/Assign^V/kernel/Assign^V/bias/Assign^dense_1/kernel/Assign^dense_1/bias/Assign^dense_2/kernel/Assign^dense_2/bias/Assign^iterations/Assign
^lr/Assign^beta_1/Assign^beta_2/Assign^decay/Assign^Variable/Assign^Variable_1/Assign^Variable_2/Assign^Variable_3/Assign^Variable_4/Assign^Variable_5/Assign^Variable_6/Assign^Variable_7/Assign^Variable_8/Assign^Variable_9/Assign^Variable_10/Assign^Variable_11/Assign^Variable_12/Assign^Variable_13/Assign^Variable_14/Assign^Variable_15/Assign^Variable_16/Assign^Variable_17/Assign^Variable_18/Assign^Variable_19/Assign^Variable_20/Assign^Variable_21/Assign^Variable_22/Assign^Variable_23/Assign^Variable_24/Assign^Variable_25/Assign^Variable_26/Assign^Variable_27/Assign^Variable_28/Assign^Variable_29/Assign^Variable_30/Assign^Variable_31/Assign
8
PlaceholderPlaceholder*
dtype0*
shape:

	Assign_48Assignbatch_normalization_1/gammaPlaceholder*
validate_shape(*.
_class$
" loc:@batch_normalization_1/gamma*
use_locking( *
T0
:
Placeholder_1Placeholder*
dtype0*
shape:

	Assign_49Assignbatch_normalization_1/betaPlaceholder_1*
validate_shape(*-
_class#
!loc:@batch_normalization_1/beta*
use_locking( *
T0
:
Placeholder_2Placeholder*
dtype0*
shape:
­
	Assign_50Assign!batch_normalization_1/moving_meanPlaceholder_2*
validate_shape(*4
_class*
(&loc:@batch_normalization_1/moving_mean*
use_locking( *
T0
:
Placeholder_3Placeholder*
dtype0*
shape:
µ
	Assign_51Assign%batch_normalization_1/moving_variancePlaceholder_3*
validate_shape(*8
_class.
,*loc:@batch_normalization_1/moving_variance*
use_locking( *
T0
>
Placeholder_4Placeholder*
dtype0*
shape
:d
}
	Assign_52Assign	h1/kernelPlaceholder_4*
validate_shape(*
_class
loc:@h1/kernel*
use_locking( *
T0
:
Placeholder_5Placeholder*
dtype0*
shape:d
y
	Assign_53Assignh1/biasPlaceholder_5*
validate_shape(*
_class
loc:@h1/bias*
use_locking( *
T0
:
Placeholder_6Placeholder*
dtype0*
shape:d
¡
	Assign_54Assignbatch_normalization_2/gammaPlaceholder_6*
validate_shape(*.
_class$
" loc:@batch_normalization_2/gamma*
use_locking( *
T0
:
Placeholder_7Placeholder*
dtype0*
shape:d

	Assign_55Assignbatch_normalization_2/betaPlaceholder_7*
validate_shape(*-
_class#
!loc:@batch_normalization_2/beta*
use_locking( *
T0
:
Placeholder_8Placeholder*
dtype0*
shape:d
­
	Assign_56Assign!batch_normalization_2/moving_meanPlaceholder_8*
validate_shape(*4
_class*
(&loc:@batch_normalization_2/moving_mean*
use_locking( *
T0
:
Placeholder_9Placeholder*
dtype0*
shape:d
µ
	Assign_57Assign%batch_normalization_2/moving_variancePlaceholder_9*
validate_shape(*8
_class.
,*loc:@batch_normalization_2/moving_variance*
use_locking( *
T0
?
Placeholder_10Placeholder*
dtype0*
shape
:dd
~
	Assign_58Assign	h2/kernelPlaceholder_10*
validate_shape(*
_class
loc:@h2/kernel*
use_locking( *
T0
;
Placeholder_11Placeholder*
dtype0*
shape:d
z
	Assign_59Assignh2/biasPlaceholder_11*
validate_shape(*
_class
loc:@h2/bias*
use_locking( *
T0
;
Placeholder_12Placeholder*
dtype0*
shape:d
¢
	Assign_60Assignbatch_normalization_3/gammaPlaceholder_12*
validate_shape(*.
_class$
" loc:@batch_normalization_3/gamma*
use_locking( *
T0
;
Placeholder_13Placeholder*
dtype0*
shape:d
 
	Assign_61Assignbatch_normalization_3/betaPlaceholder_13*
validate_shape(*-
_class#
!loc:@batch_normalization_3/beta*
use_locking( *
T0
;
Placeholder_14Placeholder*
dtype0*
shape:d
®
	Assign_62Assign!batch_normalization_3/moving_meanPlaceholder_14*
validate_shape(*4
_class*
(&loc:@batch_normalization_3/moving_mean*
use_locking( *
T0
;
Placeholder_15Placeholder*
dtype0*
shape:d
¶
	Assign_63Assign%batch_normalization_3/moving_variancePlaceholder_15*
validate_shape(*8
_class.
,*loc:@batch_normalization_3/moving_variance*
use_locking( *
T0
?
Placeholder_16Placeholder*
dtype0*
shape
:d

	Assign_64Assigndense_2/kernelPlaceholder_16*
validate_shape(*!
_class
loc:@dense_2/kernel*
use_locking( *
T0
;
Placeholder_17Placeholder*
dtype0*
shape:

	Assign_65Assigndense_2/biasPlaceholder_17*
validate_shape(*
_class
loc:@dense_2/bias*
use_locking( *
T0
?
Placeholder_18Placeholder*
dtype0*
shape
:d

	Assign_66Assigndense_1/kernelPlaceholder_18*
validate_shape(*!
_class
loc:@dense_1/kernel*
use_locking( *
T0
;
Placeholder_19Placeholder*
dtype0*
shape:

	Assign_67Assigndense_1/biasPlaceholder_19*
validate_shape(*
_class
loc:@dense_1/bias*
use_locking( *
T0
?
Placeholder_20Placeholder*
dtype0*
shape
:d
|
	Assign_68AssignV/kernelPlaceholder_20*
validate_shape(*
_class
loc:@V/kernel*
use_locking( *
T0
;
Placeholder_21Placeholder*
dtype0*
shape:
x
	Assign_69AssignV/biasPlaceholder_21*
validate_shape(*
_class
loc:@V/bias*
use_locking( *
T0"