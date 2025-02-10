// TituloMenu.js
import styles from './titulo-menu.module.css';

export default function TituloMenu(props) {
    return (
        <div>
            <h1 className={styles.titulo}>{props.children}</h1>
        </div>
    );
}