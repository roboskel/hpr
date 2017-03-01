from __future__ import division
import numpy as np
import scipy.io
import scipy.sparse.linalg as sc

def gridfit(x, y, z, xnodes, ynodes):
    # gridfit: estimates a surface on a 2d grid, based on scattered data
    #          Replicates are allowed. All methods extrapolate to the grid
    #          boundaries. Gridfit uses a modified ridge estimator to
    #          generate the surface, where the bias is toward smoothness.
    #
    #          Gridfit is not an interpolant. Its goal is a smooth surface
    #          that approximates your data, but allows you to control the
    #          amount of smoothing.
    #
    # Arguments: (input)
    #  x,y,z - vectors of equal lengths, containing arbitrary scattered data
    #          The only constraint on x and y is they cannot ALL fall on a
    #          single line in the x-y plane. Replicate points will be treated
    #          in a least squares sense.
    #
    #          ANY points containing a NaN are ignored in the estimation
    #
    #  xnodes -
    #          If xnodes is a scalar integer, then it specifies the number
    #          of equally spaced nodes between the min and max of the data.
    #
    #  ynodes -
    #          If ynodes is a scalar integer, then it specifies the number
    #          of equally spaced nodes between the min and max of the data.
    #


    #  Any UNAMBIGUOUS shortening (even down to a single letter) is
    #  valid for property names. All properties have default values,
    #  chosen (I hope) to give a reasonable result out of the box.

    #
    #
    #   'tilesize' - grids which are simply too large to solve for
    #          in one single estimation step can be built as a set
    #          of tiles. For example, a 1000x1000 grid will require
    #          the estimation of 1e6 unknowns. This is likely to
    #          require more memory (and time) than you have available.
    #          But if your data is dense enough, then you can model
    #          it locally using smaller tiles of the grid.
    #          The minimum
    #          tilesize can never be less than 3, although even this
    #          size tile is so small as to be ridiculous.
    #
    #          If your data is so sparse than some tiles contain
    #          insufficient data to model, then those tiles will
    #          be left as NaNs.
    #
    #          DEFAULT: inf
    #

    # Arguments: (output)
    #  zgrid   - (nx,ny) array containing the fitted surface

    # set defaults

    # ensure all of x,y,z,xnodes,ynodes are column vectors,
    # also drop any NaN data

    k = np.logical_or(np.isnan(x),np.isnan(y), np.isnan(z))


    xmin = min(x)
    xmax = max(x)
    ymin = min(y)
    ymax = max(y)

    nx=xnodes
    ny=ynodes
    ngrid = nx * ny

    xnodes = np.linspace(xmin,xmax,xnodes)
    ynodes = np.linspace(ymin,ymax,ynodes)

    dx = np.diff(xnodes)
    dy = np.diff(ynodes)

    # set the scaling if autoscale was on
    xscale = np.mean(dx)
    yscale = np.mean(dy)

    # check lengths of the data

    n = len(x)
    if (len(y)!=n or len(z)!=n):
        raise 'Data vectors are incompatible in size.'

    if n<3:
        raise 'Insufficient data for surface estimation.'

    # determine which cell in the array each point lies in

    indx = np.digitize(x, xnodes)
    indy = np.digitize(y, ynodes)

    # any point falling at the last node is taken to be
    # inside the last cell in x or y.

    k = np.where(indx == nx)
    #indx[k].lvalue = indx[k] - 1
    indx[k] = indx[k] - 1
    k = np.where(indy == ny)
    #indy[k].lvalue = indy[k] - 1
    indy[k] = indy[k] - 1
    ind = indy + ny * (indx - 1)
    # interpolation equations for each point
    tx = np.minimum(1, np.maximum(0, x - xnodes[indx-1])/dx[indx-1] )
    ty = np.minimum(1, np.maximum(0, y - ynodes[indy-1])/dy[indy-1] )

    # linear interpolation inside each triangle
    k = np.where(tx > ty)
    L = np.ones(n)
    L[k] = ny

    t1 = np.minimum(tx, ty)
    t2 = np.maximum(tx, ty)
    temp=np.arange(n)

    i=np.tile(temp.T,(1,3)).T
    i=np.tile(temp, 3) # for concatenation

    j=np.concatenate([ind, ind + ny + 1, ind + L])-1
    s=np.concatenate([1 - t2, t1, t2 - t1])


    A = scipy.sparse.csc_matrix((s,(i,j)),shape=(n,ngrid) )
    rhs = z

    smoothparam = 0.25
    xyRelativeStiffness = [1,1]

    # Build regularizer. Add del^4 regularizer one day.

    temp1=np.arange(0,nx)
    temp2=np.arange(1,ny-1)

    [i, j] = np.meshgrid(temp1, temp2)

    j = np.reshape(j.T, j.shape[0] * j.shape[1])
    i = np.reshape(i.T, i.shape[0] * i.shape[1])

    ind = j + ny * i
    dy1 = dy[j - 1] / yscale
    dy2 = dy[j] / yscale

    a=np.tile(ind, 3)
    b=np.concatenate([ind-1,ind,ind+1])

    sarray = np.array([-2/(dy1*(dy1+dy2)), 2/(dy1*dy2), -2/(dy2*(dy1+dy2))])
    sarrayStiff = xyRelativeStiffness[1] * np.concatenate(sarray)

    Areg1 = scipy.sparse.csc_matrix((sarrayStiff, (a, b)), shape=(ngrid, ngrid))

    [i, j] = np.meshgrid(np.arange(1,nx-1),np.arange(0,ny))

    j = np.reshape(j.T, j.shape[0] * j.shape[1])
    i = np.reshape(i.T, i.shape[0] * i.shape[1])

    ind = j + ny * i
    dx1 = dx[i - 1] / xscale
    dx2 = dx[i] / xscale

    b=np.concatenate([ind-ny,ind,ind+ny])

    sarray = np.array([-2/(dx1*(dx1+dx2)), 2/(dx1*dx2), -2/(dx2*(dx1+dx2))])
    sarrayStiff = xyRelativeStiffness[0] * np.concatenate(sarray)

    Areg2 = scipy.sparse.csc_matrix((sarrayStiff, (a, b)), shape=(ngrid, ngrid))


    Areg = scipy.sparse.vstack([Areg1, Areg2])
    nreg = Areg.shape[0]


    # Append the regularizer to the interpolation equations,
    # scaling the problem first. Use the 1-norm for speed.
    NA = np.linalg.norm(A.todense(), 1)       # IMPORTANT: Here we should use scipy's onenormest() method, which

    NR = np.linalg.norm(Areg.todense(), 1)    # is new in version 0.13.0 to find the 1-norm of sparse files.
  
    A = scipy.sparse.vstack([A, Areg * (smoothparam * NA / NR)])

    rhs = np.append(rhs, np.zeros((nreg, 1)))
    # solve the full system, with regularizer attached
    # The normal equations, solved with \.
    a = A.T * A
    b = A.T * rhs

    linearsol = np.linalg.solve(a.todense(),b) # IMPORTANT: Again we should use linear solution using sparse functions.
    zgrid = np.reshape(linearsol, (ny,nx))

    return zgrid


# ============================================
# End of main function - gridfit
# ============================================


def any(A):
#B = any(A) returns logical 1 (true) if any of the elements of A is a nonzero number or is logical 1,
#and returns logical 0 (false) if all the elements are zero.
    temp=[]
    temp=np.array(np.where(A == True))

    if (temp.size>0):
        B=True
    else:
        B=False

    return B
