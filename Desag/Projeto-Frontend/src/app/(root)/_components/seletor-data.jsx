import styles from './seletor-data.module.css';

export default function SeletorData(props) {
    return (
        <div>
        <p>{props.name}</p> 
        
        <input
            type="date" 
            className={styles.seletorData}
            name={props.name}
            value={props.value}
            onChange={props.onChange} 
        />
    </div>
    );
}
