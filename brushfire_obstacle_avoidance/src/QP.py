#!/usr/bin/env python
import numpy 
import sympy 
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Path


def subsAll(tem,subCo,value):
	tem2 = numpy.copy(tem)
	for j in xrange(0,len(tem2)):
		for k in xrange(0,len(tem2[j])):
			tem2[j,k] = (tem[j,k]).subs(subCo,value)
	return tem2
def blkdiag(a,b):
	tem = numpy.column_stack((a,numpy.zeros((len(a),len(b[0]) ))))
	tem2 = numpy.column_stack((numpy.zeros((len(b),len(a[0]) )),b))
	return numpy.concatenate((tem,tem2))
def uncon_QP(keyframe):
	deg = 9
	reldeg = 4
	time_kf = [0.,1.,2.,3.,4.]
	keyframe = [0.,2.,0.,-2.,0.]
	num_kf = len(time_kf)
	t = sympy.Symbol('t');
	polys = numpy.array([t])**numpy.arange(0,deg+1)
	poly_deriv = numpy.array([]); 
	for i in xrange(1,reldeg+1):
		tem = numpy.copy(polys)
		for j in xrange(0,len(polys)):
			tem[j] = sympy.diff(polys[j], t, i)
		poly_deriv = numpy.concatenate((poly_deriv,tem))
	poly_deriv = numpy.reshape(poly_deriv, (reldeg, -1))
	costfn_poly = poly_deriv[-1,:]

	costfn_poly =numpy.atleast_2d(costfn_poly)
	costfn_poly2 = costfn_poly.conj().transpose()*costfn_poly
	for i in xrange(0,len(costfn_poly2)):
		for j in xrange(0,len(costfn_poly2[i])):
			costfn_poly2[i,j] = sympy.integrate(costfn_poly2[i,j],t)

	H = numpy.array([[]])
	for i in xrange(0,num_kf-1):
		tem = numpy.copy(costfn_poly2)
		for j in xrange(0,len(tem)):
			for k in xrange(0,len(tem[j])):
				tem[j,k] = (tem[j,k]).subs(t,time_kf[i+1])-(tem[j,k]).subs(t,time_kf[i])
		tem2 = numpy.array([[]]);
		if i == 0:
			tem2 = numpy.copy(tem)
		else :
			tem2 = numpy.column_stack((numpy.zeros((len(tem),len(tem[0])*(i) )),tem))	
		if num_kf-i == 0:
			pass
		else :
			tem2 = numpy.column_stack((tem2,numpy.zeros((len(tem),len(tem[0])*(num_kf-2-i) ))))	

		if i == 0:
			H = numpy.copy(tem2)
		else:
			H = numpy.concatenate((H,tem2))

	A = numpy.array([[]])
	polys =numpy.atleast_2d(polys)
	derimap = numpy.concatenate((polys,poly_deriv))
	for x in xrange(0,num_kf-1):
		tem = subsAll(derimap,t,time_kf[x])
		tem2 = subsAll(derimap,t,time_kf[x+1])
		if x == 0:
			A = numpy.concatenate((tem,tem2))
		else:
			A = blkdiag(A,numpy.concatenate((tem,tem2)))
	
	totaldeg=1+reldeg;
	dgbegin = 3;
	deend = 3;
	fixdg = numpy.ones((1,num_kf))
	fixdg[0,0]=dgbegin
	fixdg[0,-1]=deend;
	freedg=totaldeg*numpy.ones((1,num_kf))-fixdg;

	M=numpy.zeros((A.shape[1-1],totaldeg*num_kf));
	for i in xrange(1,int(fixdg[0,0])+1):
		M[i-1,i-1]=1;
	for i in xrange(1,int(freedg[0,0])+1):
		M[i+int(fixdg[0,0])-1,sum(fixdg[0])+i-1]=1;

	for j in xrange(2,num_kf+1):
		for i in xrange(1,int(fixdg[0,j-1])+1):
			M[(2*j-3)*totaldeg+i-1,sum(fixdg[0,1-1:j-1])+i-1]=1;
		for i in xrange(1,int(freedg[0,j-1])+1):
			M[(2*j-3)*totaldeg+fixdg[0,j-1]+i-1,sum(fixdg[0])+sum(freedg[0,1-1:j-1])+i-1]=1;
		if j != num_kf:
			for i in xrange(1,totaldeg+1):
				M[(2*j-2)*totaldeg+i-1:(2*j-1)*totaldeg,:] = M[(2*j-3)*totaldeg+i-1:(2*j-2)*totaldeg,:]
	
	M = M.conj().transpose()
	
	Df1 = numpy.atleast_2d(numpy.array([keyframe[0]]))
	Df2 = numpy.atleast_2d(numpy.array([0,0]))
	Df3 = numpy.atleast_2d(numpy.array(keyframe[1:]))
	DD = numpy.column_stack((Df1,Df2))
	DD = numpy.column_stack((DD,Df3))
	Df = numpy.column_stack((DD,Df2))
	Df = Df.conj().transpose()

	mDf = numpy.asmatrix(Df)
	mM = numpy.asmatrix(M)
	mH = numpy.asmatrix(H)
	mA = numpy.asmatrix(A)

	minvA = numpy.linalg.inv(mA)
	R = mM*minvA.H*mH*minvA*mM.H
	Rfp=R[1-1:sum(fixdg[0]),sum(fixdg[0])+1-1:];
	Rpp=R[sum(fixdg[0])+1-1:,sum(fixdg[0])+1-1:];
	Dp=-numpy.linalg.inv(Rpp)*Rfp.H*mDf;
	Draw=mM.H*numpy.concatenate((mDf,Dp));
	uncons_coef = minvA*Draw;

	uncons_coef.shape = (num_kf-1,deg+1)
	polynomial = uncons_coef
	print polynomial.shape
	print polynomial


	alltime = numpy.array([[]])
	alltraj = numpy.array([[]])
	for x in xrange(0,len(time_kf)-1):
		time = numpy.linspace(time_kf[x],time_kf[x+1],100)
		time = numpy.atleast_2d(time)
		alltime = numpy.column_stack((alltime,time))
		for i in xrange(0,len(time[0])):
			tem = subsAll(derimap,t,time[0,i])
			tem2 = subsAll(polynomial[x,:].conj().transpose(),t,time[0,i])
			val = numpy.asmatrix(tem)*numpy.asmatrix(tem2)
			if x == 0 and i == 0:
				alltraj = val
			else :
				alltraj = numpy.column_stack((alltraj,val))
	print alltraj.shape
		
    	
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('QP', anonymous=True)
    pub = rospy.Publisher('Uncon_Way', String, queue_size=10)
    rospy.Subscriber("way_point", String, callback)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()
    # spin() simply keeps python from ex7iting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
    