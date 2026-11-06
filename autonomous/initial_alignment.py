"""Bounded scan-to-map initialization using the supplied start cell and bearing.

Search is only used before motion. AMCL handles all subsequent localization.
No simulator state is read. NumPy is the only numerical dependency.
"""
import math
import numpy as np

class InitialAlignment:
    def __init__(self,msg):
        self.info=msg.info
        occupied=np.asarray(msg.data).reshape(msg.info.height,msg.info.width)>=50
        self.distance=np.full(occupied.shape,.20,dtype=np.float32)
        radius=math.ceil(.20/msg.info.resolution)
        h,w=occupied.shape
        for dy in range(-radius,radius+1):
            for dx in range(-radius,radius+1):
                distance=math.hypot(dx,dy)*msg.info.resolution
                if distance>.20:continue
                src=(slice(max(0,-dy),min(h,h-dy)),slice(max(0,-dx),min(w,w-dx)))
                dst=(slice(max(0,dy),min(h,h+dy)),slice(max(0,dx),min(w,w+dx)))
                np.minimum(self.distance[dst],np.where(occupied[src],distance,.20),out=self.distance[dst])

    def align(self,points,centre,heading):
        points=np.asarray(points)
        origin=self.info.origin
        q=origin.orientation
        angle=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        c,s=math.cos(angle),math.sin(angle)
        rotation=np.array([[c,s],[-s,c]])
        h,w=self.distance.shape
        def search(xs,ys,headings):
            xy=np.array(np.meshgrid(xs,ys)).reshape(2,-1).T
            best=(float('inf'),None)
            for yaw in headings:
                c,s=math.cos(yaw),math.sin(yaw)
                rotated=points@np.array([[c,s],[-s,c]])
                world=xy[:,None,:]+rotated[None,:,:]
                local=(world-np.array([origin.position.x,origin.position.y]))@rotation.T
                pixels=np.floor(local/self.info.resolution).astype(int)
                ix,iy=pixels[:,:,0],pixels[:,:,1]
                valid=(ix>=0)&(ix<w)&(iy>=0)&(iy<h)
                distances=np.where(valid,self.distance[np.clip(iy,0,h-1),np.clip(ix,0,w-1)],.20)
                # Ignore the worst 20%: an unknown object must not dominate the fit.
                count=max(1,int(.8*distances.shape[1]))
                score=np.mean(np.partition(distances,count-1,axis=1)[:,:count]**2,axis=1)
                i=int(np.argmin(score))
                if score[i]<best[0]:best=(float(score[i]),(float(xy[i,0]),float(xy[i,1]),float(yaw)))
            return best
        offsets=np.linspace(-.20,.20,11)
        score,pose=search(centre[0]+offsets,centre[1]+offsets,heading+np.radians(np.linspace(-45,45,31)))
        fine=np.linspace(-.04,.04,17)
        xs=np.clip(pose[0]+fine,centre[0]-.20,centre[0]+.20)
        ys=np.clip(pose[1]+fine,centre[1]-.20,centre[1]+.20)
        angles=np.clip(pose[2]+np.radians(np.linspace(-3,3,13)),heading-math.pi/4,heading+math.pi/4)
        score,pose=search(xs,ys,angles)
        return pose,math.sqrt(score)
