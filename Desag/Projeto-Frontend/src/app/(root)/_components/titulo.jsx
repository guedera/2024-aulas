// Titulo.js
import styles from './titulo.module.css';

export default function Titulo( props ) {
    return (
        <div>
            <h1 className={styles.titulo}>{props.children}</h1>
        </div>
    );
}