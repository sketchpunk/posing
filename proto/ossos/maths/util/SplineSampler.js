// #region IMPORTS
import SplineSample from './SplineSample.js';
import { Vec3 }     from '../../ossos.min.js';
// #endregion


export default class SplineSampler{
    // #region MAIN
    items     = []; // Each sample of the curve
    arcLength = 0;  // Total Length of the Spline
    
    constructor(){}
    // #endregion

    // #region BUILD

    fromSpline( s, sampCnt = 5 ){
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~        
        const eIdx          = sampCnt - 1;
        this.arcLength      = 0;
        this.items.length   = 0;

        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        let itm = new SplineSample();
        s.at( itm.time, itm.pos, itm.tangent );
        this.items.push( itm );

        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        for( let i=1; i <= eIdx; i++ ){
            itm      = new SplineSample();
            itm.time = i / eIdx;
            s.at( itm.time, itm.pos, itm.tangent );

            // @ts-ignore Object is possibly 'undefined'.ts(2532) :: at(-1)
            itm.inc  = Vec3.dist( itm.pos, this.items.at(-1).pos );
            itm.dist = ( this.arcLength += itm.inc );
            
            this.items.push( itm );
        }

        return this;
    }
    // #endregion

    // #region GETTERS
    // Get lerped sample at a specific distance along the spline sampling
    atDist( dist, out=new SplineSample() ){
        let a;
        let b;
        let t;

        // console.log( 'atDist', dist, this.arcLength, this.items.length );

        if( dist <= 0 ){
            // Skip to first two items
            a = this.items[0];
            b = this.items[1];
            t = 0;
        }else if( dist >= this.arcLength ){
            // Skip to the last two items
            a = this.items.at(-2);
            b = this.items.at(-1);
            t = 1;
        }else{
            // BinarySearch: Find the first item that is greater then seek value
            let imid;
            let imin = 0;
            let imax = this.items.length - 1;

            while( imin < imax ){                     // Once Min Crosses or Equals Max, Stop Loop.
                imid = ( imin + imax ) >>> 1;         // Compute Mid Index
                if( dist < this.items[ imid ].dist )
                    imax = imid;      // value is LT seek, use mid as new Max Range
                else               
                    imin = imid + 1;  // value is GTE seek, move min to one after mid to make the cross fail happen
            }

            // console.log( imax, dist );
            // console.log( '----', this.items[ imax-1 ].dist )
            // console.log( '----', this.items[ imax ].dist )

            a = this.items[ imax-1 ];                    // Get the two samples where seek is between
            b = this.items[ imax ];
            t = ( dist - a.dist ) / ( b.dist - a.dist ); // Compute T between the two samples
        }
        
        // console.log( a, b, t );

        out.fromLerp( a, b, t );        

        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // for( let i=b; i >= a; i-- ){
        //     if( this.lenAry[ i ] < len ){
        //         let tt	= ( len - this.lenAry[ i ] ) / this.incAry[ i+1 ];          // Normalize the Search Length   ( x-a / b-a )
        //         let ttt	= this.timeAry[ i ] * (1-tt) + this.timeAry[ i+1 ] * tt;    // Interpolate the Curve Time between two points
        //         return ttt / this.curveCnt;                                         // Since time saved as as Curve# + CurveT, Normalize it based on total time which is curve count
        //     }
        // }
        return out;
    }


    // #endregion
}