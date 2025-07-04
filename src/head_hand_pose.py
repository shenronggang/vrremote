from scipy.spatial.transform import Rotation
import numpy as np

def rt2homo(r_zyx,t):
    R = np.eye(4)
    R[:3, :3] = Rotation.from_euler("zyx", r_zyx).as_dcm()
    R[:3,-1]=t
    return R

def home2rt(homo):
    t=homo[:3,-1]
    r=Rotation.from_dcm(homo[:3,:3])
    euler= r.as_euler('zyx')
    return euler,t 

def fast_mat_inv(mat):
    ret = np.eye(4)
    ret[:3, :3] = mat[:3, :3].T
    ret[:3, 3] = -mat[:3, :3].T @ mat[:3, 3]
    return ret

def process(head_t,head_r,left_hand_t,left_hand_r,right_hand_t,right_hand_r):
    '''
    convert base2hand to head2hand
    '''
    base2head=rt2homo(head_r,head_t)
    base2left_hand=rt2homo(left_hand_r,left_hand_t)
    base2right_hand=rt2homo(right_hand_r,right_hand_t)

    head2base=fast_mat_inv(base2head)
    head2left_hand=head2base @ base2left_hand
    head2right_hand=head2base @ base2right_hand


    new_left_hand_t=head2left_hand[:3,-1]
    r=Rotation.from_dcm(head2left_hand[:3,:3])
    new_left_hand_r= r.as_euler('zyx')

    new_right_hand_t=head2right_hand[:3,-1]
    r=Rotation.from_dcm(head2right_hand[:3,:3])
    new_right_hand_r= r.as_euler('zyx')

    return new_left_hand_r,new_left_hand_t,new_right_hand_r,new_right_hand_t

if __name__=="__main__":
    head_t=[1,1,1]
    head_r=[-1.57,0,0]
    l_hand_t=[0,1,1]
    l_hand_r=[1.57,0,0]
    r_hand_t=[2,1,1]
    r_hand_r=[-1.57,0,0]
    new_left_hand_r,new_left_hand_t,new_right_hand_r,new_right_hand_t=process(head_t,head_r,l_hand_t,l_hand_r,r_hand_t,r_hand_r)
    print(new_left_hand_r,new_left_hand_t,new_right_hand_r,new_right_hand_t)


