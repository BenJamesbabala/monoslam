/* 
    MonoSLAM:  A vision based SLAM program
    Based upon SceneLib, by Andrew Davison ( http://www.doc.ic.ac.uk/~ajd )
    Copyright (C) 2006  Bob Mottram

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA 
*/

using System;
using System.Collections.Generic;
using System.Text;

namespace SceneLibrary
{  
    /// <summary>
    /// A base class for feature identifiers. A feature identifier is something in
    /// the real world that identifies that feature uniquely, for example an image
    /// patch. Unique identifiers for features must inherit from this, but having this
    /// base class means that the inherited classes must know how to delete themselves
    /// if features are deleted. When we compare two identifiers the test will check
    /// just the pointer data members so each identifier will be unique.
    /// </summary>
    public class ExtraData
    {   
    }

    /// <summary>
    /// Call a pointer to ExtraData an Identifier for ease of understanding
    /// elsewhere in the code. An Identifier is a pointer to something in the real
    /// world that identifies a feature uniquely.
    /// </summary>
    //public class Identifier : ExtraData
    //{   
    //}

}
