// #region IMPORTS
import { Vec3 } from '../ossos.min.js';
// #endregion

// #region POINTS
export const PointType = Object.freeze({
    Point   : 0,
    Control : 1,
});

export class Point{
    attrib = {};
    type   = PointType.Point;
    index  = -1;
    pos    = new Vec3();
    constructor( pos, t = PointType.Point ){
        this.pos.copy( pos );
        this.type = t;
    }

    setPos( v ){ this.pos.copy( v ); }
}
// #endregion

// #region CURVE
export class Curve{
    points = [];   // Array of points in the proper order of the specific curve
    attrib = {};   // Used to extra data per curve, NURBS for example
    constructor(){}

    // Clone a curve that may extent this curve
    extendClone( start, end ){
        const curv  = new Curve();
        curv.points = this.points.slice( start, end );
        curv.attrib = {...this.attrib};
        return curv;
    }
}
// #endregion

export class Spline{
    // #region MAIN
    points  = [];  // Flatten out all the points for
    curves  = [];  // Collection of points per curve
    _isLoop = false;            // Is the spline closed? Meaning should the ends be treated as another curve
    // #endregion

    // #region GETTERS / SETTERS
    get pointCount(){ return this.points.length; }
    get curveCount(){ return this.curves.length; }
    // #endregion

    // #region MANAGE POINTS

    // @ts-ignore 6133
    appendCurve( pnts ){ console.log( 'Spline.appendCurve is not implemented' ); return this; }

    /** Update point position */
    setPos( idx, pos ){
        // @ts-ignore ts(2532) :: at(-1) is fine
        this.points.at(idx).pos.copy( pos );
        return this;
    }

    // #endregion

    // #region ABSTRACT METHODS
    // @ts-ignore 6133
    at( t, pos, dxdy, dxdy2){}

    // @ts-ignore 6133
    atCurve( cIdx, t, pos, dxdy, dxdy2 ){}
    // #endregion

    // #region HELPERS

    /** Return [ CurveIndex, CurveT ] : Take t over spline & compute curve's index and lerp time */
    _tOfCurves( t ){ 
        const rtn = [0,0]; // if t <= 0
        if( t >= 1 ){
            rtn[0] = this.curves.length - 1;
            rtn[1] = 1;
        }else if( t > 0 ){
            const f = this.curves.length * t;
            rtn[0] = Math.floor( f );
            rtn[1] = f - rtn[0];
        }

        return rtn;
    }

    // #endregion
}