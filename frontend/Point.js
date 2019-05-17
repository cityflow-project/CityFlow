class Point {
    constructor(x, y) {
        if (arguments.length == 1) {
            this.x = x[0];
            this.y = x[1];
        } else if (arguments.length == 2) {
            this.x = x;
            this.y = y;
        }   
    }
    
    getLength() {
        return Math.sqrt(this.x*this.x + this.y*this.y);
    }
    
    getAngle() {
        return this.getAngleInRadians() * 180 / Math.PI;
    }

    getAngleInRadians() {
        return Math.atan2(this.y, this.x);
    }

    add(point) {
        return new Point(this.x + point.x, this.y + point.y)
    }

    directTo(point) {
        let x = point.x - this.x;
        let y = point.y - this.y;
        let length = Math.sqrt(x*x + y*y);
        let scale = length !== 0 ? 1 / length : 0;
        return new Point(x * scale, y * scale);
    }

    distanceTo(point) {
        let x = point.x - this.x;
        let y = point.y - this.y;
        return Math.sqrt(x*x + y*y);
    }

    moveAlong(direct, length) {
        return new Point(this.x + direct.x * length, this.y + direct.y * length);
    }

    moveAlongDirectTo(point, length) {
        let direct = this.directTo(point);
        return new Point(this.x + direct.x * length, this.y + direct.y * length);
    }

    rotate(angle) {
        angle = angle * Math.PI / 180;
        let sin = Math.sin(angle);
        let cos = Math.cos(angle);
        return new Point(this.x * cos - this.y * sin, this.x * sin + this.y * cos);
    }
}